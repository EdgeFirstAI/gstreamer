// SPDX-License-Identifier: Apache-2.0

use gst::glib;
use gst::prelude::*;

mod imp;

glib::wrapper! {
    pub struct DummyTensor(ObjectSubclass<imp::DummyTensor>)
        @extends gst_base::BaseTransform, gst::Element, gst::Object;
}

pub fn register(plugin: &gst::Plugin) -> Result<(), glib::BoolError> {
    gst::Element::register(
        Some(plugin),
        "edgefirstdummytensor",
        gst::Rank::NONE,
        DummyTensor::static_type(),
    )
}
