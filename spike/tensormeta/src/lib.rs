// SPDX-License-Identifier: Apache-2.0

use gst::glib;

mod dummytensor;

fn plugin_init(plugin: &gst::Plugin) -> Result<(), glib::BoolError> {
    dummytensor::register(plugin)
}

gst::plugin_define!(
    edgefirstspike,
    env!("CARGO_PKG_DESCRIPTION"),
    plugin_init,
    concat!(env!("CARGO_PKG_VERSION"), "-", env!("COMMIT_ID")),
    "Apache 2.0",
    env!("CARGO_PKG_NAME"),
    env!("CARGO_PKG_NAME"),
    env!("CARGO_PKG_REPOSITORY"),
    env!("BUILD_REL_DATE")
);
