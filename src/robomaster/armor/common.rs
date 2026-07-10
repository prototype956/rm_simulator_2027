#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum ArmorType {
    Small = 0,
    Large = 1,
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum ArmorLabel {
    Sentry = 0,
    One = 1,
    Two = 2,
    Three = 3,
    Four = 4,
    Five = 5,
    Outpost = 6,
    Base = 7,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum SmallArmorLabel {
    Sentry,
    One,
    Base,
    Outpost,
    Two,
    Three,
    Four,
    Five,
}

impl SmallArmorLabel {
    pub const fn label(self) -> ArmorLabel {
        match self {
            Self::One => ArmorLabel::One,
            Self::Base => ArmorLabel::Base,
            Self::Sentry => ArmorLabel::Sentry,
            Self::Outpost => ArmorLabel::Outpost,
            Self::Two => ArmorLabel::Two,
            Self::Three => ArmorLabel::Three,
            Self::Four => ArmorLabel::Four,
            Self::Five => ArmorLabel::Five,
        }
    }
}

impl From<SmallArmorLabel> for ArmorLabel {
    fn from(label: SmallArmorLabel) -> Self {
        label.label()
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum LargeArmorLabel {
    One,
}

impl LargeArmorLabel {
    pub const fn label(self) -> ArmorLabel {
        match self {
            Self::One => ArmorLabel::One,
        }
    }
}

impl From<LargeArmorLabel> for ArmorLabel {
    fn from(label: LargeArmorLabel) -> Self {
        label.label()
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum ArmorSpec {
    Small(SmallArmorLabel),
    Large(LargeArmorLabel),
}

impl ArmorSpec {
    pub const fn armor_type(self) -> ArmorType {
        match self {
            Self::Small(_) => ArmorType::Small,
            Self::Large(_) => ArmorType::Large,
        }
    }

    pub const fn label(self) -> ArmorLabel {
        match self {
            Self::Small(label) => label.label(),
            Self::Large(label) => label.label(),
        }
    }

    pub const fn sticker_slots(self) -> &'static [ArmorStickerSlot] {
        match self {
            Self::Small(_) => &SMALL_ARMOR_STICKER_SLOTS,
            Self::Large(_) => &LARGE_ARMOR_STICKER_SLOTS,
        }
    }
}

impl From<SmallArmorLabel> for ArmorSpec {
    fn from(label: SmallArmorLabel) -> Self {
        Self::Small(label)
    }
}

impl From<LargeArmorLabel> for ArmorSpec {
    fn from(label: LargeArmorLabel) -> Self {
        Self::Large(label)
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub struct ArmorStickerSlot {
    pub label: ArmorLabel,
    pub name_suffix: &'static str,
}

pub const SMALL_ARMOR_STICKER_SLOTS: [ArmorStickerSlot; 7] = [
    ArmorStickerSlot {
        label: ArmorLabel::Base,
        name_suffix: "B",
    },
    ArmorStickerSlot {
        label: ArmorLabel::Sentry,
        name_suffix: "G",
    },
    ArmorStickerSlot {
        label: ArmorLabel::Outpost,
        name_suffix: "O",
    },
    ArmorStickerSlot {
        label: ArmorLabel::Two,
        name_suffix: "2",
    },
    ArmorStickerSlot {
        label: ArmorLabel::Three,
        name_suffix: "3",
    },
    ArmorStickerSlot {
        label: ArmorLabel::Four,
        name_suffix: "4",
    },
    ArmorStickerSlot {
        label: ArmorLabel::Five,
        name_suffix: "5",
    },
];

pub const LARGE_ARMOR_STICKER_SLOTS: [ArmorStickerSlot; 5] = [
    ArmorStickerSlot {
        label: ArmorLabel::One,
        name_suffix: "1",
    },
    ArmorStickerSlot {
        label: ArmorLabel::Base,
        name_suffix: "3",
    },
    ArmorStickerSlot {
        label: ArmorLabel::Four,
        name_suffix: "4",
    },
    ArmorStickerSlot {
        label: ArmorLabel::Five,
        name_suffix: "5",
    },
    ArmorStickerSlot {
        label: ArmorLabel::Base,
        name_suffix: "B",
    },
];

impl ArmorLabel {
    pub fn sequence_small() -> &'static [ArmorLabel; 8] {
        &[
            ArmorLabel::Sentry,
            ArmorLabel::One,
            ArmorLabel::Two,
            ArmorLabel::Three,
            ArmorLabel::Four,
            ArmorLabel::Outpost,
            ArmorLabel::Base,
            ArmorLabel::Five,
        ]
    }

    pub fn index_from_small(label: ArmorLabel) -> usize {
        match label {
            ArmorLabel::Sentry => 0,
            ArmorLabel::One => 1,
            ArmorLabel::Two => 2,
            ArmorLabel::Three => 3,
            ArmorLabel::Four => 4,
            ArmorLabel::Outpost => 5,
            ArmorLabel::Base => 6,
            ArmorLabel::Five => 7,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn armor_spec_preserves_legacy_type_and_label() {
        let cases = [
            (
                ArmorSpec::Small(SmallArmorLabel::Sentry),
                ArmorType::Small,
                ArmorLabel::Sentry,
            ),
            (
                ArmorSpec::Small(SmallArmorLabel::Outpost),
                ArmorType::Small,
                ArmorLabel::Outpost,
            ),
            (
                ArmorSpec::Large(LargeArmorLabel::One),
                ArmorType::Large,
                ArmorLabel::One,
            ),
        ];

        for (spec, armor_type, label) in cases {
            assert_eq!(spec.armor_type(), armor_type);
            assert_eq!(spec.label(), label);
        }
    }

    #[test]
    fn debug_sequence_and_indexes_keep_legacy_order() {
        assert_eq!(
            ArmorLabel::sequence_small(),
            &[
                ArmorLabel::Sentry,
                ArmorLabel::One,
                ArmorLabel::Two,
                ArmorLabel::Three,
                ArmorLabel::Four,
                ArmorLabel::Outpost,
                ArmorLabel::Base,
                ArmorLabel::Five,
            ]
        );

        assert_eq!(ArmorLabel::index_from_small(ArmorLabel::Sentry), 0);
        assert_eq!(ArmorLabel::index_from_small(ArmorLabel::One), 1);
        assert_eq!(ArmorLabel::index_from_small(ArmorLabel::Two), 2);
        assert_eq!(ArmorLabel::index_from_small(ArmorLabel::Three), 3);
        assert_eq!(ArmorLabel::index_from_small(ArmorLabel::Four), 4);
        assert_eq!(ArmorLabel::index_from_small(ArmorLabel::Outpost), 5);
        assert_eq!(ArmorLabel::index_from_small(ArmorLabel::Base), 6);
        assert_eq!(ArmorLabel::index_from_small(ArmorLabel::Five), 7);
        assert_eq!(ArmorLabel::index_from_small(ArmorLabel::Base), 8);
    }

    #[test]
    fn sticker_slot_tables_keep_asset_suffixes() {
        assert_eq!(
            ArmorSpec::Small(SmallArmorLabel::Outpost).sticker_slots(),
            &[
                ArmorStickerSlot {
                    label: ArmorLabel::Base,
                    name_suffix: "B",
                },
                ArmorStickerSlot {
                    label: ArmorLabel::Sentry,
                    name_suffix: "G",
                },
                ArmorStickerSlot {
                    label: ArmorLabel::Outpost,
                    name_suffix: "O",
                },
                ArmorStickerSlot {
                    label: ArmorLabel::Two,
                    name_suffix: "2",
                },
                ArmorStickerSlot {
                    label: ArmorLabel::Three,
                    name_suffix: "3",
                },
                ArmorStickerSlot {
                    label: ArmorLabel::Four,
                    name_suffix: "4",
                },
                ArmorStickerSlot {
                    label: ArmorLabel::Five,
                    name_suffix: "5",
                },
            ]
        );

        assert_eq!(
            ArmorSpec::Large(LargeArmorLabel::One).sticker_slots(),
            &[
                ArmorStickerSlot {
                    label: ArmorLabel::One,
                    name_suffix: "1",
                },
                ArmorStickerSlot {
                    label: ArmorLabel::Three,
                    name_suffix: "3",
                },
                ArmorStickerSlot {
                    label: ArmorLabel::Four,
                    name_suffix: "4",
                },
                ArmorStickerSlot {
                    label: ArmorLabel::Five,
                    name_suffix: "5",
                },
                ArmorStickerSlot {
                    label: ArmorLabel::Base,
                    name_suffix: "B",
                },
            ]
        );
    }
}
