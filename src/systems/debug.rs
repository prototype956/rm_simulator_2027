use bevy::prelude::*;
use bevy::render::view::screenshot::{Capturing, Screenshot, save_to_disk};
use bevy::window::{CursorIcon, SystemCursorIcon, Window};

use crate::components::{SlapperInfantry, SubscribeAutoAim};
use crate::robomaster::prelude::{Armor, ArmorStickerSelection};
use crate::statistic::ProjectileStatistics;
use crate::systems::ControllerState;

/// Only this node receives the keyboard/controller help string.
#[derive(Component)]
pub struct HelpText;

fn create_help_text(
    auto_aim: bool,
    stats: &ProjectileStatistics,
    controller: &ControllerState,
) -> Text {
    format!(
        "auto-aim={} bullets={} armor-hits={} hit-rate={:.2} rune-hits={} darts={}\ncontroller={} mode={} gyro={} remote-gyro={}\n{}",
        if auto_aim { "ON " } else { "OFF" },
        stats.bullet_launch_count,
        stats.armor_hit_count,
        stats.armor_hit_rate(),
        stats.rune_hit_count,
        stats.dart_launch_count,
        controller.help_source(),
        controller.help_mode(),
        if controller.controlled_chassis_spin() {
            "ON"
        } else {
            "OFF"
        },
        if controller.remote_chassis_spin() {
            "ON"
        } else {
            "OFF"
        },
        controller.help_controls()
    )
    .into()
}

pub fn spawn_text(commands: &mut Commands) {
    commands.spawn((
        HelpText,
        // The HUD plugin reveals this only after binding it to a window camera.
        Visibility::Hidden,
        Text::new(""),
        Node {
            position_type: PositionType::Absolute,
            bottom: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
    ));
}

pub fn update_help_text(
    mut text: Query<&mut Text, With<HelpText>>,
    auto_aim: Res<SubscribeAutoAim>,
    stats: Res<ProjectileStatistics>,
    controller: Res<ControllerState>,
    round: Option<Res<crate::robomaster::combat::reset::TrainingRound>>,
    clock: Option<Res<Time<Fixed>>>,
) {
    for mut text in text.iter_mut() {
        *text = create_help_text(
            auto_aim.load(std::sync::atomic::Ordering::Acquire),
            &stats,
            &controller,
        );
        if let (Some(round), Some(clock)) = (&round, &clock) {
            text.0.push_str(&format!(
                "\nround={} sim={:.1}s | R Reset",
                round.id,
                clock
                    .elapsed()
                    .saturating_sub(round.started_at)
                    .as_secs_f64()
            ));
        }
    }
}

pub fn change_appearance(
    keyboard: Res<ButtonInput<KeyCode>>,
    selections: Query<&mut ArmorStickerSelection, With<SlapperInfantry>>,
    owned: Query<&mut Armor, With<SlapperInfantry>>,
) {
    if keyboard.pressed(KeyCode::ShiftLeft) && keyboard.just_pressed(KeyCode::KeyC) {
        let mut n_type = None;
        for mut selection in selections {
            let new_typ = selection.advance_debug_sequence();
            n_type = Some(new_typ);
        }
        if let Some(n_type) = n_type {
            for mut own in owned {
                own.label = n_type;
            }
        }
    }
}

pub fn screenshot_on_f2(mut commands: Commands, mut counter: Local<u32>) {
    let path = format!("./screenshot-{}.png", *counter);
    *counter += 1;
    commands
        .spawn(Screenshot::primary_window())
        .observe(save_to_disk(path));
}

pub fn screenshot_saving(
    mut commands: Commands,
    screenshot_saving: Query<Entity, With<Capturing>>,
    window: Single<Entity, With<Window>>,
) {
    match screenshot_saving.iter().count() {
        0 => {
            commands.entity(*window).remove::<CursorIcon>();
        }
        x if x > 0 => {
            commands
                .entity(*window)
                .insert(CursorIcon::from(SystemCursorIcon::Progress));
        }
        _ => {}
    }
}
