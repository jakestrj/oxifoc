#[derive(Clone, PartialEq, ::femtopb::Message)]
pub struct WireOptions<'a> {
    #[femtopb(string, tag = 1)]
    pub project: &'a str,
    #[femtopb(unknown_fields)]
    pub unknown_fields: femtopb::UnknownFields<'a>,
}
#[derive(
    Clone,
    Copy,
    Debug,
    PartialEq,
    Eq,
    Hash,
    PartialOrd,
    Ord,
    ::femtopb::Enumeration
)]
#[repr(i32)]
#[derive(Default)]
pub enum PayloadSource {
    /// HV BMS master
    #[default]
    HvBms = 0,
    Rotary = 2,
    Oxifoc = 3,
    /// PC Host.
    Host = 1,
}
impl PayloadSource {
    /// String value of the enum field names used in the ProtoBuf definition.
    ///
    /// The values are not transformed in any way and thus are considered stable
    /// (if the ProtoBuf definition does not change) and safe for programmatic use.
    pub fn as_str_name(&self) -> &'static str {
        match self {
            Self::HvBms => "HV_BMS",
            Self::Rotary => "ROTARY",
            Self::Oxifoc => "OXIFOC",
            Self::Host => "HOST",
        }
    }
    /// Creates an enum from field names used in the ProtoBuf definition.
    pub fn from_str_name(value: &str) -> ::core::option::Option<Self> {
        match value {
            "HV_BMS" => Some(Self::HvBms),
            "ROTARY" => Some(Self::Rotary),
            "OXIFOC" => Some(Self::Oxifoc),
            "HOST" => Some(Self::Host),
            _ => None,
        }
    }
}
