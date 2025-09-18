#pragma once
// NTRIP caster (v1 SOURCE)
static constexpr const char*  CASTER_HOST = "url.to.caster";
static constexpr uint16_t     CASTER_PORT = 1234;
// Some casters want "SOURCE pw mount", others accept "pw /mount". Toggle flag if needed.
static constexpr const char*  NTRIP_MOUNTPOINT = "MountPoint";
static constexpr bool         MOUNTPOINT_WITH_SLASH = false;
static constexpr const char*  NTRIP_SOURCE_PW  = "Password";