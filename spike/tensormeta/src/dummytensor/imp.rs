// SPDX-License-Identifier: Apache-2.0

//! Attaches one zero-filled float32 tensor of dims `[1, 84, 8400]` (col-major)
//! to every video buffer and advertises it in the `tensors` caps field.

use std::sync::{LazyLock, Mutex};

use gst::glib;
use gst::prelude::*;
use gst::subclass::prelude::*;
use gst_analytics::{Tensor, TensorDataType, TensorDimOrder, TensorMeta};
use gst_base::subclass::prelude::*;

const DEFAULT_TENSOR_ID: &str = "yolo-v8-out";
const DIMS: [usize; 3] = [1, 84, 8400];

static CAT: LazyLock<gst::DebugCategory> = LazyLock::new(|| {
    gst::DebugCategory::new(
        "edgefirstdummytensor",
        gst::DebugColorFlags::empty(),
        Some("Dummy tensor meta element"),
    )
});

struct Settings {
    tensor_id: String,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            tensor_id: DEFAULT_TENSOR_ID.into(),
        }
    }
}

#[derive(Default)]
pub struct DummyTensor {
    settings: Mutex<Settings>,
}

impl DummyTensor {
    fn tensors_field(tensor_id: &str) -> gst::Structure {
        let dims = gst::Array::new(DIMS.iter().map(|d| *d as i32));
        let tensor = gst::Caps::builder("tensor/strided")
            .field("tensor-id", tensor_id)
            .field("dims", dims)
            .field("dims-order", "col-major")
            .field("type", "float32")
            .build();
        gst::Structure::builder("tensorgroups")
            .field(tensor_id, gst::UniqueList::new([tensor]))
            .build()
    }
}

#[glib::object_subclass]
impl ObjectSubclass for DummyTensor {
    const NAME: &'static str = "EdgeFirstDummyTensor";
    type Type = super::DummyTensor;
    type ParentType = gst_base::BaseTransform;
}

impl ObjectImpl for DummyTensor {
    fn properties() -> &'static [glib::ParamSpec] {
        static PROPS: LazyLock<Vec<glib::ParamSpec>> = LazyLock::new(|| {
            vec![glib::ParamSpecString::builder("tensor-id")
                .nick("Tensor ID")
                .blurb("Tensor id set on the tensor and in the tensors caps field")
                .default_value(Some(DEFAULT_TENSOR_ID))
                .mutable_ready()
                .build()]
        });
        PROPS.as_ref()
    }

    fn set_property(&self, _id: usize, value: &glib::Value, pspec: &glib::ParamSpec) {
        if pspec.name() == "tensor-id" {
            self.settings.lock().unwrap().tensor_id = value
                .get::<Option<String>>()
                .unwrap()
                .unwrap_or_else(|| DEFAULT_TENSOR_ID.into());
        }
    }

    fn property(&self, _id: usize, pspec: &glib::ParamSpec) -> glib::Value {
        match pspec.name() {
            "tensor-id" => self.settings.lock().unwrap().tensor_id.to_value(),
            _ => unimplemented!(),
        }
    }
}

impl GstObjectImpl for DummyTensor {}

impl ElementImpl for DummyTensor {
    fn metadata() -> Option<&'static gst::subclass::ElementMetadata> {
        static META: LazyLock<gst::subclass::ElementMetadata> = LazyLock::new(|| {
            gst::subclass::ElementMetadata::new(
                "EdgeFirst dummy tensor",
                "Filter/Video",
                "Attaches a zero-filled tensor meta to every buffer",
                "Au-Zone Technologies",
            )
        });
        Some(&*META)
    }

    fn pad_templates() -> &'static [gst::PadTemplate] {
        static TEMPLATES: LazyLock<Vec<gst::PadTemplate>> = LazyLock::new(|| {
            let caps = gst::Caps::new_empty_simple("video/x-raw");
            vec![
                gst::PadTemplate::new("src", gst::PadDirection::Src, gst::PadPresence::Always, &caps)
                    .unwrap(),
                gst::PadTemplate::new("sink", gst::PadDirection::Sink, gst::PadPresence::Always, &caps)
                    .unwrap(),
            ]
        });
        TEMPLATES.as_ref()
    }
}

impl BaseTransformImpl for DummyTensor {
    const MODE: gst_base::subclass::BaseTransformMode =
        gst_base::subclass::BaseTransformMode::AlwaysInPlace;
    const PASSTHROUGH_ON_SAME_CAPS: bool = false;
    const TRANSFORM_IP_ON_PASSTHROUGH: bool = false;

    fn transform_caps(
        &self,
        direction: gst::PadDirection,
        caps: &gst::Caps,
        filter: Option<&gst::Caps>,
    ) -> Option<gst::Caps> {
        let tensor_id = self.settings.lock().unwrap().tensor_id.clone();
        let mut out = caps.clone();
        for s in out.make_mut().iter_mut() {
            match direction {
                gst::PadDirection::Sink => s.set("tensors", Self::tensors_field(&tensor_id)),
                _ => s.remove_field("tensors"),
            }
        }
        gst::debug!(CAT, imp = self, "{direction:?} {caps} -> {out}");
        // The filter rejects incompatible caps but is not intersected into `tensors`.
        Some(match filter {
            Some(f) if !f.can_intersect(&out) => gst::Caps::new_empty(),
            _ => out,
        })
    }

    fn transform_ip(&self, buf: &mut gst::BufferRef) -> Result<gst::FlowSuccess, gst::FlowError> {
        let tensor_id = self.settings.lock().unwrap().tensor_id.clone();
        let nbytes = DIMS.iter().product::<usize>() * std::mem::size_of::<f32>();
        let data = gst::Buffer::from_mut_slice(vec![0u8; nbytes]);
        let tensor = Tensor::new_simple(
            glib::Quark::from_str(&tensor_id),
            TensorDataType::Float32,
            data,
            TensorDimOrder::ColMajor,
            &DIMS,
        );
        let mut meta = TensorMeta::add(buf);
        meta.set(glib::Slice::from([tensor]));
        Ok(gst::FlowSuccess::Ok)
    }
}
