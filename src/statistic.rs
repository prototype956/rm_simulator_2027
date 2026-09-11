use bevy::prelude::*;

#[derive(Resource, Default, Reflect)]
#[reflect(Resource)]
pub struct ProjectileStatistics {
    pub bullet_launch_count: u64,
    pub armor_hit_count: u64,
    pub rune_hit_count: u32,
    pub dart_launch_count: u32,
}

impl ProjectileStatistics {
    pub fn increase_bullet_launch(&mut self) {
        self.bullet_launch_count = self.bullet_launch_count.saturating_add(1);
    }

    pub fn increase_armor_hit(&mut self) {
        self.armor_hit_count = self.armor_hit_count.saturating_add(1);
    }

    pub fn increase_rune_hit(&mut self) {
        self.rune_hit_count = self.rune_hit_count.saturating_add(1);
    }

    pub fn increase_dart_launch(&mut self) {
        self.dart_launch_count = self.dart_launch_count.saturating_add(1);
    }

    pub fn armor_hit_rate(&self) -> f64 {
        if self.bullet_launch_count == 0 {
            return 0.0;
        }
        (self.armor_hit_count as f64) / (self.bullet_launch_count as f64)
    }
}
