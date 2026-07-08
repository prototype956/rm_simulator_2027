use bevy::prelude::*;

#[derive(Default)]
pub struct MetalFxTemporalPlugin;

impl Plugin for MetalFxTemporalPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<MetalFxTemporalUpscaling>();

        #[cfg(target_os = "macos")]
        macos::build(app);
    }
}

#[derive(Component, Reflect, Clone)]
#[reflect(Component, Clone)]
#[require(
    bevy::render::camera::TemporalJitter,
    bevy::render::camera::MipBias,
    bevy::core_pipeline::prepass::DepthPrepass,
    bevy::core_pipeline::prepass::MotionVectorPrepass
)]
pub struct MetalFxTemporalUpscaling {
    pub scale_factor: f32,
    pub frame_generation: bool,
    pub reset: bool,
}

impl Default for MetalFxTemporalUpscaling {
    fn default() -> Self {
        Self {
            scale_factor: 2.0,
            frame_generation: cfg!(target_os = "macos"),
            reset: true,
        }
    }
}

#[cfg(target_os = "macos")]
mod macos {
    use super::MetalFxTemporalUpscaling;
    use bevy::camera::{Camera, Camera3d, CameraMainTextureUsages, MainPassResolutionOverride};
    use bevy::core_pipeline::{
        Core3d, Core3dSystems,
        core_3d::CORE_3D_DEPTH_FORMAT,
        prepass::{
            DepthPrepass, MOTION_VECTOR_PREPASS_FORMAT, MotionVectorPrepass, ViewPrepassTextures,
        },
    };
    use bevy::diagnostic::FrameCount;
    use bevy::math::{UVec2, Vec4Swizzles};
    use bevy::prelude::*;
    use bevy::render::{
        Extract, ExtractSchedule, MainWorld, Render, RenderApp, RenderSystems,
        camera::{MipBias, TemporalJitter},
        render_resource::{
            Extent3d, Origin3d, TexelCopyTextureInfo, Texture, TextureAspect, TextureDescriptor,
            TextureDimension, TextureFormat, TextureUsages,
        },
        renderer::{RenderContext, RenderDevice, ViewQuery},
        sync_world::RenderEntity,
        view::{ExtractedView, ViewTarget, prepare_view_targets},
    };
    use objc2::{
        Message,
        rc::Retained,
        runtime::{AnyClass, ProtocolObject},
    };
    use objc2_metal::{MTLPixelFormat, MTLTexture};
    use objc2_metal_fx::{
        MTLFXFrameInterpolatableScaler, MTLFXFrameInterpolator, MTLFXFrameInterpolatorBase,
        MTLFXFrameInterpolatorDescriptor, MTLFXTemporalScaler, MTLFXTemporalScalerBase,
        MTLFXTemporalScalerDescriptor,
    };
    use std::{
        ffi::CStr,
        sync::atomic::{AtomicBool, Ordering},
    };

    const MIN_UPSCALE_FACTOR: f32 = 1.01;
    const JITTER_SEQUENCE: [Vec2; 8] = [
        Vec2::ZERO,
        Vec2::new(0.0, -0.16666666),
        Vec2::new(-0.25, 0.16666669),
        Vec2::new(0.25, -0.3888889),
        Vec2::new(-0.375, -0.055555552),
        Vec2::new(0.125, 0.2777778),
        Vec2::new(-0.125, -0.2777778),
        Vec2::new(0.375, 0.055555582),
    ];

    pub fn build(app: &mut App) {
        if let Some(render_app) = app.get_sub_app_mut(RenderApp) {
            render_app
                .init_resource::<MetalFxFrameDeltaSeconds>()
                .add_systems(
                    ExtractSchedule,
                    (extract_metalfx_frame_delta, extract_metalfx_temporal),
                )
                .add_systems(
                    Render,
                    prepare_metalfx_temporal
                        .in_set(RenderSystems::PrepareViews)
                        .before(prepare_view_targets),
                )
                .add_systems(
                    Core3d,
                    metalfx_temporal_upscale.in_set(Core3dSystems::EarlyPostProcess),
                );
        }
    }

    #[derive(Component)]
    struct MetalFxTemporalRenderContext {
        scaler: objc2::rc::Retained<ProtocolObject<dyn MTLFXTemporalScaler>>,
        frame_interpolator: Option<objc2::rc::Retained<ProtocolObject<dyn MTLFXFrameInterpolator>>>,
        frame_generation_textures: Option<FrameGenerationTextures>,
        frame_history_valid: AtomicBool,
        render_resolution: UVec2,
        upscaled_resolution: UVec2,
        color_format: TextureFormat,
        frame_generation_requested: bool,
    }

    struct FrameGenerationTextures {
        current_color: Texture,
        previous_color: Texture,
    }

    #[derive(Resource, Default)]
    struct MetalFxFrameDeltaSeconds(f32);

    unsafe impl Send for MetalFxTemporalRenderContext {}
    unsafe impl Sync for MetalFxTemporalRenderContext {}

    fn extract_metalfx_temporal(
        mut commands: Commands,
        mut main_world: ResMut<MainWorld>,
        cleanup_query: Query<Has<MetalFxTemporalUpscaling>>,
    ) {
        let mut cameras_3d = main_world.query::<(
            RenderEntity,
            &Camera,
            &Projection,
            Option<&mut MetalFxTemporalUpscaling>,
        )>();

        for (entity, camera, camera_projection, mut metalfx) in cameras_3d.iter_mut(&mut main_world)
        {
            let mut entity_commands = commands
                .get_entity(entity)
                .expect("Camera entity wasn't synced.");
            if let Some(metalfx) = metalfx.as_deref_mut()
                && camera.is_active
                && camera_projection.is_perspective()
                && metalfx.scale_factor >= MIN_UPSCALE_FACTOR
            {
                entity_commands.insert(metalfx.clone());
                metalfx.reset = false;
            } else if cleanup_query.get(entity) == Ok(true) {
                entity_commands.remove::<(
                    MetalFxTemporalUpscaling,
                    MetalFxTemporalRenderContext,
                    MainPassResolutionOverride,
                )>();
            }
        }
    }

    fn extract_metalfx_frame_delta(
        mut delta: ResMut<MetalFxFrameDeltaSeconds>,
        time: Extract<Res<Time>>,
    ) {
        delta.0 = time.delta_secs().max(f32::EPSILON);
    }

    fn prepare_metalfx_temporal(
        mut query: Query<
            (
                Entity,
                &ExtractedView,
                &MetalFxTemporalUpscaling,
                &mut Camera3d,
                &mut CameraMainTextureUsages,
                &mut TemporalJitter,
                &mut MipBias,
                Option<&MetalFxTemporalRenderContext>,
            ),
            (
                With<Camera3d>,
                With<TemporalJitter>,
                With<DepthPrepass>,
                With<MotionVectorPrepass>,
            ),
        >,
        render_device: Res<RenderDevice>,
        frame_count: Res<FrameCount>,
        mut commands: Commands,
    ) {
        for (
            entity,
            view,
            metalfx,
            mut camera_3d,
            mut camera_main_texture_usages,
            mut temporal_jitter,
            mut mip_bias,
            context,
        ) in &mut query
        {
            let upscaled_resolution = view.viewport.zw();
            let render_resolution = render_resolution(upscaled_resolution, metalfx.scale_factor);
            if render_resolution == upscaled_resolution {
                commands
                    .entity(entity)
                    .remove::<(MetalFxTemporalRenderContext, MainPassResolutionOverride)>();
                continue;
            }

            camera_main_texture_usages.0 |= TextureUsages::STORAGE_BINDING;
            camera_main_texture_usages.0 |= TextureUsages::COPY_DST;

            let mut depth_texture_usages = TextureUsages::from(camera_3d.depth_texture_usages);
            depth_texture_usages |= TextureUsages::TEXTURE_BINDING;
            camera_3d.depth_texture_usages = depth_texture_usages.into();

            temporal_jitter.offset =
                JITTER_SEQUENCE[(frame_count.0 as usize) % JITTER_SEQUENCE.len()];
            mip_bias.0 = -metalfx.scale_factor.log2();

            let needs_new_context = context
                .map(|context| {
                    context.render_resolution != render_resolution
                        || context.upscaled_resolution != upscaled_resolution
                        || context.color_format != view.target_format
                        || context.frame_generation_requested != metalfx.frame_generation
                })
                .unwrap_or(true);

            if !needs_new_context {
                commands
                    .entity(entity)
                    .insert(MainPassResolutionOverride(render_resolution));
                continue;
            }

            match MetalFxTemporalRenderContext::new(
                &render_device,
                render_resolution,
                upscaled_resolution,
                view.target_format,
                metalfx.frame_generation,
            ) {
                Some(context) => {
                    commands
                        .entity(entity)
                        .insert((context, MainPassResolutionOverride(render_resolution)));
                }
                None => {
                    warn!(
                        "MetalFX temporal scaler unavailable for {:?}; rendering full resolution",
                        view.target_format
                    );
                    commands
                        .entity(entity)
                        .remove::<(MetalFxTemporalRenderContext, MainPassResolutionOverride)>();
                }
            }
        }
    }

    fn metalfx_temporal_upscale(
        view: ViewQuery<(
            &MetalFxTemporalUpscaling,
            &MetalFxTemporalRenderContext,
            &MainPassResolutionOverride,
            &TemporalJitter,
            &ViewTarget,
            &ViewPrepassTextures,
            &Projection,
        )>,
        frame_delta: Res<MetalFxFrameDeltaSeconds>,
        mut ctx: RenderContext,
    ) {
        let (
            metalfx,
            metalfx_context,
            resolution_override,
            temporal_jitter,
            view_target,
            prepass_textures,
            projection,
        ) = view.into_inner();

        let (Some(prepass_depth_texture), Some(prepass_motion_vectors_texture)) =
            (&prepass_textures.depth, &prepass_textures.motion_vectors)
        else {
            return;
        };

        let view_target = view_target.post_process_write();
        let render_resolution = resolution_override.0;
        let frame_generation = metalfx_context
            .frame_interpolator
            .as_ref()
            .zip(metalfx_context.frame_generation_textures.as_ref())
            .filter(|_| metalfx.frame_generation);

        let Some(color_texture) = retain_metal_texture(view_target.source_texture) else {
            return;
        };
        let Some(depth_texture) = retain_metal_texture(&prepass_depth_texture.texture.texture)
        else {
            return;
        };
        let Some(motion_texture) =
            retain_metal_texture(&prepass_motion_vectors_texture.texture.texture)
        else {
            return;
        };
        let Some(destination_texture) = retain_metal_texture(view_target.destination_texture)
        else {
            return;
        };

        if let Some((frame_interpolator, frame_generation_textures)) = frame_generation {
            let Some(current_color_texture) =
                retain_metal_texture(&frame_generation_textures.current_color)
            else {
                return;
            };
            let Some(previous_color_texture) =
                retain_metal_texture(&frame_generation_textures.previous_color)
            else {
                return;
            };
            let has_history =
                metalfx_context.frame_history_valid.load(Ordering::Relaxed) && !metalfx.reset;

            unsafe {
                encode_temporal_scaler(
                    metalfx,
                    metalfx_context,
                    render_resolution,
                    temporal_jitter,
                    color_texture.as_ref(),
                    depth_texture.as_ref(),
                    motion_texture.as_ref(),
                    current_color_texture.as_ref(),
                    &mut ctx,
                );

                if has_history {
                    let (near_plane, far_plane, field_of_view, aspect_ratio) =
                        perspective_parameters(projection, metalfx_context.upscaled_resolution);
                    frame_interpolator.setColorTexture(Some(current_color_texture.as_ref()));
                    frame_interpolator.setPrevColorTexture(Some(previous_color_texture.as_ref()));
                    frame_interpolator.setDepthTexture(Some(depth_texture.as_ref()));
                    frame_interpolator.setMotionTexture(Some(motion_texture.as_ref()));
                    frame_interpolator.setOutputTexture(Some(destination_texture.as_ref()));
                    frame_interpolator.setMotionVectorScaleX(-(render_resolution.x as f32));
                    frame_interpolator.setMotionVectorScaleY(-(render_resolution.y as f32));
                    frame_interpolator.setDeltaTime(frame_delta.0);
                    frame_interpolator.setNearPlane(near_plane);
                    frame_interpolator.setFarPlane(far_plane);
                    frame_interpolator.setFieldOfView(field_of_view);
                    frame_interpolator.setAspectRatio(aspect_ratio);
                    frame_interpolator.setJitterOffsetX(-temporal_jitter.offset.x);
                    frame_interpolator.setJitterOffsetY(-temporal_jitter.offset.y);
                    frame_interpolator.setShouldResetHistory(false);
                    frame_interpolator.setDepthReversed(true);

                    ctx.command_encoder()
                        .as_hal_mut::<wgpu::hal::api::Metal, _, _>(|encoder| {
                            let Some(encoder) = encoder else {
                                return;
                            };
                            let Some(command_buffer) = encoder.raw_command_buffer() else {
                                return;
                            };
                            frame_interpolator.encodeToCommandBuffer(command_buffer);
                        });
                } else {
                    copy_texture(
                        &mut ctx,
                        &frame_generation_textures.current_color,
                        view_target.destination_texture,
                        metalfx_context.upscaled_resolution,
                    );
                }
            }

            copy_texture(
                &mut ctx,
                &frame_generation_textures.current_color,
                &frame_generation_textures.previous_color,
                metalfx_context.upscaled_resolution,
            );
            metalfx_context
                .frame_history_valid
                .store(true, Ordering::Relaxed);
            return;
        }

        unsafe {
            encode_temporal_scaler(
                metalfx,
                metalfx_context,
                render_resolution,
                temporal_jitter,
                color_texture.as_ref(),
                depth_texture.as_ref(),
                motion_texture.as_ref(),
                destination_texture.as_ref(),
                &mut ctx,
            );
        }
    }

    unsafe fn encode_temporal_scaler(
        metalfx: &MetalFxTemporalUpscaling,
        metalfx_context: &MetalFxTemporalRenderContext,
        render_resolution: UVec2,
        temporal_jitter: &TemporalJitter,
        color_texture: &ProtocolObject<dyn MTLTexture>,
        depth_texture: &ProtocolObject<dyn MTLTexture>,
        motion_texture: &ProtocolObject<dyn MTLTexture>,
        output_texture: &ProtocolObject<dyn MTLTexture>,
        ctx: &mut RenderContext,
    ) {
        unsafe {
            metalfx_context
                .scaler
                .setInputContentWidth(render_resolution.x as _);
            metalfx_context
                .scaler
                .setInputContentHeight(render_resolution.y as _);
            metalfx_context.scaler.setColorTexture(Some(color_texture));
            metalfx_context.scaler.setDepthTexture(Some(depth_texture));
            metalfx_context
                .scaler
                .setMotionTexture(Some(motion_texture));
            metalfx_context
                .scaler
                .setOutputTexture(Some(output_texture));
            metalfx_context
                .scaler
                .setJitterOffsetX(-temporal_jitter.offset.x);
            metalfx_context
                .scaler
                .setJitterOffsetY(-temporal_jitter.offset.y);
            metalfx_context
                .scaler
                .setMotionVectorScaleX(-(render_resolution.x as f32));
            metalfx_context
                .scaler
                .setMotionVectorScaleY(-(render_resolution.y as f32));
            metalfx_context.scaler.setReset(metalfx.reset);
            metalfx_context.scaler.setDepthReversed(true);

            ctx.command_encoder()
                .as_hal_mut::<wgpu::hal::api::Metal, _, _>(|encoder| {
                    let Some(encoder) = encoder else {
                        return;
                    };
                    let Some(command_buffer) = encoder.raw_command_buffer() else {
                        return;
                    };
                    metalfx_context.scaler.encodeToCommandBuffer(command_buffer);
                });
        }
    }

    impl MetalFxTemporalRenderContext {
        fn new(
            render_device: &RenderDevice,
            render_resolution: UVec2,
            upscaled_resolution: UVec2,
            color_format: TextureFormat,
            frame_generation_requested: bool,
        ) -> Option<Self> {
            let color_format_mtl = metal_pixel_format(color_format)?;
            let motion_format_mtl = metal_pixel_format(MOTION_VECTOR_PREPASS_FORMAT)?;
            let depth_format_mtl = metal_pixel_format(CORE_3D_DEPTH_FORMAT)?;

            let (scaler, frame_interpolator) = unsafe {
                let device_guard = render_device
                    .wgpu_device()
                    .as_hal::<wgpu::hal::api::Metal>()?;
                let device = device_guard.raw_device().as_ref();
                if !objc_class_exists(b"MTLFXTemporalScalerDescriptor\0")
                    || !MTLFXTemporalScalerDescriptor::supportsDevice(device)
                {
                    return None;
                }
                let descriptor = MTLFXTemporalScalerDescriptor::new();
                descriptor.setColorTextureFormat(color_format_mtl);
                descriptor.setOutputTextureFormat(color_format_mtl);
                descriptor.setDepthTextureFormat(depth_format_mtl);
                descriptor.setMotionTextureFormat(motion_format_mtl);
                descriptor.setInputWidth(render_resolution.x as _);
                descriptor.setInputHeight(render_resolution.y as _);
                descriptor.setOutputWidth(upscaled_resolution.x as _);
                descriptor.setOutputHeight(upscaled_resolution.y as _);
                descriptor.setAutoExposureEnabled(true);
                descriptor.setRequiresSynchronousInitialization(false);
                let scaler = descriptor.newTemporalScalerWithDevice(device)?;

                let frame_interpolator = if frame_generation_requested
                    && objc_class_exists(b"MTLFXFrameInterpolatorDescriptor\0")
                    && MTLFXFrameInterpolatorDescriptor::supportsDevice(device)
                {
                    let frame_descriptor = MTLFXFrameInterpolatorDescriptor::new();
                    frame_descriptor.setColorTextureFormat(color_format_mtl);
                    frame_descriptor.setOutputTextureFormat(color_format_mtl);
                    frame_descriptor.setDepthTextureFormat(depth_format_mtl);
                    frame_descriptor.setMotionTextureFormat(motion_format_mtl);
                    frame_descriptor.setInputWidth(upscaled_resolution.x as _);
                    frame_descriptor.setInputHeight(upscaled_resolution.y as _);
                    frame_descriptor.setOutputWidth(upscaled_resolution.x as _);
                    frame_descriptor.setOutputHeight(upscaled_resolution.y as _);
                    let temporal_scaler: &ProtocolObject<dyn MTLFXTemporalScaler> = scaler.as_ref();
                    let interpolatable_scaler: &ProtocolObject<dyn MTLFXFrameInterpolatableScaler> =
                        temporal_scaler.as_ref();
                    frame_descriptor.setScaler(Some(interpolatable_scaler));
                    frame_descriptor.newFrameInterpolatorWithDevice(device)
                } else {
                    None
                };

                (scaler, frame_interpolator)
            };

            if frame_generation_requested && frame_interpolator.is_none() {
                warn!("MetalFX frame interpolator unavailable; using temporal upscaling only");
            }

            let frame_generation_textures = frame_interpolator.as_ref().map(|_| {
                FrameGenerationTextures::new(render_device, upscaled_resolution, color_format)
            });

            Some(Self {
                scaler,
                frame_interpolator,
                frame_generation_textures,
                frame_history_valid: AtomicBool::new(false),
                render_resolution,
                upscaled_resolution,
                color_format,
                frame_generation_requested,
            })
        }
    }

    impl FrameGenerationTextures {
        fn new(
            render_device: &RenderDevice,
            upscaled_resolution: UVec2,
            color_format: TextureFormat,
        ) -> Self {
            Self {
                current_color: create_frame_generation_texture(
                    render_device,
                    upscaled_resolution,
                    color_format,
                    "metalfx_frame_generation_current_color",
                ),
                previous_color: create_frame_generation_texture(
                    render_device,
                    upscaled_resolution,
                    color_format,
                    "metalfx_frame_generation_previous_color",
                ),
            }
        }
    }

    fn create_frame_generation_texture(
        render_device: &RenderDevice,
        resolution: UVec2,
        format: TextureFormat,
        label: &'static str,
    ) -> Texture {
        render_device.create_texture(&TextureDescriptor {
            label: Some(label),
            size: texture_extent(resolution),
            mip_level_count: 1,
            sample_count: 1,
            dimension: TextureDimension::D2,
            format,
            usage: TextureUsages::STORAGE_BINDING
                | TextureUsages::TEXTURE_BINDING
                | TextureUsages::COPY_SRC
                | TextureUsages::COPY_DST,
            view_formats: &[],
        })
    }

    fn copy_texture(
        ctx: &mut RenderContext,
        source: &Texture,
        destination: &Texture,
        resolution: UVec2,
    ) {
        ctx.command_encoder().copy_texture_to_texture(
            texture_copy_info(source),
            texture_copy_info(destination),
            texture_extent(resolution),
        );
    }

    fn texture_copy_info(texture: &Texture) -> TexelCopyTextureInfo<'_> {
        TexelCopyTextureInfo {
            texture,
            mip_level: 0,
            origin: Origin3d::ZERO,
            aspect: TextureAspect::All,
        }
    }

    fn texture_extent(resolution: UVec2) -> Extent3d {
        Extent3d {
            width: resolution.x,
            height: resolution.y,
            depth_or_array_layers: 1,
        }
    }

    fn perspective_parameters(projection: &Projection, resolution: UVec2) -> (f32, f32, f32, f32) {
        let aspect_ratio = resolution.x as f32 / resolution.y as f32;
        match projection {
            Projection::Perspective(projection) => (
                projection.near,
                projection.far,
                projection.fov.to_degrees(),
                aspect_ratio,
            ),
            _ => (0.1, 10000.0, 60.0, aspect_ratio),
        }
    }

    fn retain_metal_texture(texture: &Texture) -> Option<Retained<ProtocolObject<dyn MTLTexture>>> {
        let texture = unsafe { texture.as_hal::<wgpu::hal::api::Metal>() }?;
        Some(texture.raw_handle().retain())
    }

    fn render_resolution(upscaled: UVec2, scale_factor: f32) -> UVec2 {
        let scale_factor = scale_factor.max(MIN_UPSCALE_FACTOR);
        UVec2::new(
            ((upscaled.x as f32 / scale_factor).round() as u32).max(1),
            ((upscaled.y as f32 / scale_factor).round() as u32).max(1),
        )
        .min(upscaled - UVec2::ONE)
    }

    fn metal_pixel_format(format: TextureFormat) -> Option<MTLPixelFormat> {
        match format {
            TextureFormat::Rgba8Unorm => Some(MTLPixelFormat::RGBA8Unorm),
            TextureFormat::Rgba8UnormSrgb => Some(MTLPixelFormat::RGBA8Unorm_sRGB),
            TextureFormat::Bgra8Unorm => Some(MTLPixelFormat::BGRA8Unorm),
            TextureFormat::Bgra8UnormSrgb => Some(MTLPixelFormat::BGRA8Unorm_sRGB),
            TextureFormat::Rgba16Float => Some(MTLPixelFormat::RGBA16Float),
            TextureFormat::Rg16Float => Some(MTLPixelFormat::RG16Float),
            TextureFormat::Depth32Float => Some(MTLPixelFormat::Depth32Float),
            _ => None,
        }
    }

    fn objc_class_exists(name: &'static [u8]) -> bool {
        let name =
            CStr::from_bytes_with_nul(name).expect("Objective-C class name must be NUL-terminated");
        AnyClass::get(name).is_some()
    }
}
