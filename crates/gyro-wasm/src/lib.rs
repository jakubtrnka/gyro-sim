//! JavaScript boundary. All physics remains in gyro-core; returned data is owned.
//! Aircraft axes: x forward, y right, z down. Quaternions: xyzw, Hamilton,
//! world-from-component. Angles/rates are radians and rad/s, never degrees/RPM.

use gyro_core::*;
use serde::{Serialize, de::DeserializeOwned};
use wasm_bindgen::prelude::*;

const MAX_SAFE_INTEGER: u64 = 9_007_199_254_740_991;
const MAX_BATCH: u32 = 32_768;

#[wasm_bindgen(typescript_custom_section)]
const TYPES: &str = include_str!("bindings.d.ts");

#[derive(Serialize)]
struct BoundaryError {
    kind: &'static str,
    message: String,
    step: Option<u64>,
}

fn error(kind: &'static str, message: impl ToString, step: Option<u64>) -> JsValue {
    BoundaryError {
        kind,
        message: message.to_string(),
        step,
    }
    .serialize(&serde_wasm_bindgen::Serializer::json_compatible())
    .unwrap_or_else(|e| JsValue::from_str(&e.to_string()))
}

fn core_error(e: SimulationError) -> JsValue {
    let (kind, step) = match e {
        SimulationError::InvalidConfiguration(_) => ("invalid-configuration", None),
        SimulationError::InvalidInput(_) => ("invalid-input", None),
        SimulationError::NumericalFailure { step, .. } => ("numerical-failure", step),
    };
    error(kind, e, step)
}

fn decode<T: DeserializeOwned>(value: JsValue) -> Result<T, JsValue> {
    // Inputs are JSON-shaped commands/configurations, not per-frame snapshots.
    // serde-wasm-bindgen's struct deserializer visits only declared fields and
    // cannot enforce deny_unknown_fields. JSON enforces that schema and its
    // recursion limit, rejects cycles, and maps non-finite numbers to invalid nulls.
    // float_roundtrip preserves finite input f64 values through this boundary.
    let text = js_sys::JSON::stringify(&value)
        .map_err(|_| error("invalid-input", "input must be an acyclic JSON value", None))?
        .as_string()
        .ok_or_else(|| error("invalid-input", "input must be a JSON value", None))?;
    serde_json::from_str(&text).map_err(|e| error("invalid-input", e, None))
}

fn encode<T: Serialize>(value: &T) -> Result<JsValue, JsValue> {
    value
        .serialize(&serde_wasm_bindgen::Serializer::json_compatible())
        .map_err(|e| error("serialization", e, None))
}

#[derive(Serialize)]
struct Definition {
    config: SimulationConfig,
    initial: InitialState,
}

#[wasm_bindgen(unchecked_return_type = "Definition")]
pub fn preset_definition(name: &str) -> Result<JsValue, JsValue> {
    let preset = match name {
        "Attitude" => Preset::Attitude,
        "Directional" => Preset::Directional,
        "TurnIndicator" => Preset::TurnIndicator,
        _ => return Err(error("invalid-input", "unknown preset", None)),
    };
    encode(&Definition {
        config: SimulationConfig::for_preset(preset),
        initial: InitialState::for_preset(preset),
    })
}

#[wasm_bindgen]
pub struct GyroSimulation {
    inner: Simulation,
}

#[wasm_bindgen]
impl GyroSimulation {
    #[wasm_bindgen(constructor)]
    pub fn new(
        #[wasm_bindgen(unchecked_param_type = "SimulationConfig")] config: JsValue,
        #[wasm_bindgen(unchecked_param_type = "InitialState")] initial: JsValue,
    ) -> Result<GyroSimulation, JsValue> {
        Ok(Self {
            inner: Simulation::new(decode(config)?, decode(initial)?).map_err(core_error)?,
        })
    }

    #[wasm_bindgen(unchecked_return_type = "Snapshot")]
    pub fn snapshot(&self) -> Result<JsValue, JsValue> {
        encode(&self.inner.snapshot())
    }

    #[wasm_bindgen(unchecked_return_type = "Snapshot")]
    pub fn step(&mut self) -> Result<JsValue, JsValue> {
        self.check_steps(1)?;
        encode(&self.inner.step().map_err(core_error)?)
    }

    #[wasm_bindgen(unchecked_return_type = "AdvanceResult")]
    pub fn advance_steps(&mut self, count: f64) -> Result<JsValue, JsValue> {
        if !count.is_finite() || count < 0.0 || count.fract() != 0.0 || count > f64::from(MAX_BATCH)
        {
            return Err(error(
                "invalid-input",
                "step count must be an integer between 0 and 32,768",
                None,
            ));
        }
        let count = count as u32;
        self.check_steps(count)?;
        encode(&self.inner.advance_steps(count).map_err(core_error)?)
    }

    #[wasm_bindgen(unchecked_return_type = "CommandRecord")]
    pub fn queue(
        &mut self,
        #[wasm_bindgen(unchecked_param_type = "Command")] command: JsValue,
    ) -> Result<JsValue, JsValue> {
        encode(&self.inner.queue(decode(command)?).map_err(core_error)?)
    }

    #[wasm_bindgen(unchecked_return_type = "CommandRecord")]
    pub fn enqueue(
        &mut self,
        #[wasm_bindgen(unchecked_param_type = "CommandRecord")] record: JsValue,
    ) -> Result<JsValue, JsValue> {
        let record: CommandRecord = decode(record)?;
        if record.step_index > MAX_SAFE_INTEGER {
            return Err(error(
                "invalid-input",
                "step index exceeds JavaScript safe integer range",
                None,
            ));
        }
        encode(&self.inner.enqueue(record).map_err(core_error)?)
    }

    #[wasm_bindgen(unchecked_return_type = "Snapshot")]
    pub fn reset(
        &mut self,
        #[wasm_bindgen(unchecked_param_type = "InitialState")] initial: JsValue,
    ) -> Result<JsValue, JsValue> {
        encode(&self.inner.reset(decode(initial)?).map_err(core_error)?)
    }

    #[wasm_bindgen(unchecked_return_type = "Replay")]
    pub fn replay(&self) -> Result<JsValue, JsValue> {
        encode(&self.inner.replay())
    }

    /// Validate all records and return a queued run at step zero. The caller
    /// advances to replay.end_step in bounded batches, yielding between batches
    /// so a large replay cannot synchronously lock the browser UI.
    pub fn prepare_replay(
        #[wasm_bindgen(unchecked_param_type = "Replay")] replay: JsValue,
    ) -> Result<GyroSimulation, JsValue> {
        let replay: Replay = decode(replay)?;
        if replay.version != Replay::VERSION {
            return Err(error("invalid-input", "unsupported replay version", None));
        }
        if replay.end_step > MAX_SAFE_INTEGER
            || replay.commands.len() > 50_000
            || replay
                .commands
                .iter()
                .any(|r| r.step_index > MAX_SAFE_INTEGER)
        {
            return Err(error(
                "invalid-input",
                "replay exceeds safe step range or 50,000-command limit",
                None,
            ));
        }
        let mut inner = Simulation::new(replay.config, replay.initial).map_err(core_error)?;
        for record in replay.commands {
            inner.enqueue(record).map_err(core_error)?;
        }
        Ok(Self { inner })
    }
}

impl GyroSimulation {
    fn check_steps(&self, count: u32) -> Result<(), JsValue> {
        if count > MAX_BATCH
            || self.inner.snapshot().step_index > MAX_SAFE_INTEGER - u64::from(count)
        {
            return Err(error(
                "invalid-input",
                "batch exceeds 32,768 steps or JavaScript safe step range",
                None,
            ));
        }
        Ok(())
    }
}
