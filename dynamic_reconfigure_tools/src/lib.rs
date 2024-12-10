use roslibrust::ros1::Publisher;
use roslibrust_util::dynamic_reconfigure;
use std::sync::{Arc, Mutex};
// use tokio::sync::mpsc;

pub fn get_server_name(partial_server_name: &str) -> String {
    format!("/{}/set_parameters", &partial_server_name,)
        .replace("//", "/")
        .replace("set_parameters/set_parameters", "set_parameters")
}

#[derive(Debug)]
pub enum ParamType {
    INT,
    DOUBLE,
    BOOL,
    STR,
}

pub fn raw_value_to_request(
    raw_value: &str,
    name: &str,
) -> (dynamic_reconfigure::ReconfigureRequest, ParamType) {
    let param_type;

    let name = name.to_string();
    let mut req = dynamic_reconfigure::ReconfigureRequest::default();
    // TODO(lucasw) with this setup can't set a string to 'true' or a number, need to prefix
    // it with other text to make it fail through
    // could require a type be provided on the command line instead
    if let Ok(value) = raw_value.parse::<i32>() {
        let int_param = dynamic_reconfigure::IntParameter { name, value };
        req.config.ints.push(int_param);
        param_type = ParamType::INT;
    } else if let Ok(value) = raw_value.parse::<f64>() {
        let double_param = dynamic_reconfigure::DoubleParameter { name, value };
        req.config.doubles.push(double_param);
        param_type = ParamType::DOUBLE;
    } else if let Ok(value) = raw_value.parse::<bool>() {
        let bool_param = dynamic_reconfigure::BoolParameter { name, value };
        req.config.bools.push(bool_param);
        param_type = ParamType::BOOL;
    } else {
        let value = raw_value.to_string();
        let str_param = dynamic_reconfigure::StrParameter { name, value };
        req.config.strs.push(str_param);
        param_type = ParamType::STR;
    }
    // TODO(lucasw) enum params

    tracing::info!("{req:?}");
    (req, param_type)
}

pub struct DynamicReconfigure {
    config_description: dynamic_reconfigure::ConfigDescription,
    config_state: Arc<Mutex<dynamic_reconfigure::Config>>,
    update_pub: Publisher<dynamic_reconfigure::Config>,
    description_pub: Publisher<dynamic_reconfigure::ConfigDescription>,
    _server_handle: roslibrust::ros1::ServiceServer,
    // this doesn't work, panics
    // pub update_receiver: mpsc::Receiver<bool>,
    do_update_pub: Arc<Mutex<bool>>,
}

impl DynamicReconfigure {
    pub async fn new(
        nh: &roslibrust::ros1::NodeHandle,
        full_node_name: &str,
    ) -> Result<Self, anyhow::Error> {
        let mut config_description = dynamic_reconfigure::ConfigDescription::default();
        let mut config_state = dynamic_reconfigure::Config::default();
        {
            let group = dynamic_reconfigure::Group {
                name: "Default".to_string(),
                ..Default::default()
            };

            let mut group_state = dynamic_reconfigure::GroupState {
                name: group.name.clone(),
                state: true,
                ..Default::default()
            };
            group_state.name = group.name.clone();
            group_state.state = true;
            config_description.min.groups.push(group_state.clone());
            config_description.max.groups.push(group_state.clone());
            config_description.dflt.groups.push(group_state.clone());
            config_description.groups.push(group);

            config_state.groups.push(group_state);
        }
        let config_state = Arc::new(Mutex::new(config_state));
        tracing::info!("Initial config: {config_state:?}");

        // TODO(lucasw) need these to be latched, but maybe just regularly publishing will work
        let latch = true;
        let update_pub: Publisher<dynamic_reconfigure::Config> = nh
            .advertise(&format!("{}/parameter_updates", full_node_name), 3, latch)
            .await?;
        // let do_update_pub = Arc::new(Mutex::new(true));
        let description_pub: Publisher<dynamic_reconfigure::ConfigDescription> = nh
            .advertise(
                &format!("{}/parameter_descriptions", full_node_name),
                3,
                latch,
            )
            .await?;

        let do_update_pub = Arc::new(Mutex::new(true));

        // can't use these within the closure
        // let (update_sender, update_receiver) = mpsc::channel(4);
        let config_state_copy = config_state.clone();
        let do_update_pub_copy = do_update_pub.clone();

        let server_fn = move |request: dynamic_reconfigure::ReconfigureRequest| {
            tracing::info!("{request:?}");

            // TODO(lucasw) store the changed value in the main DR config_state
            let mut config = config_state_copy.lock().unwrap();

            // TODO(lucasw) need a hashmap as base structure so don't have to do double for loops
            // - then convert it to and from Config as needed
            'outer: for req_val in request.config.bools {
                for val in &mut config.bools {
                    if req_val.name == val.name && val.value != req_val.value {
                        // TODO(lucasw) also need to clip to min max values
                        tracing::info!(
                            "set '{}' value '{}' to '{}'",
                            val.name,
                            val.value,
                            req_val.value
                        );
                        val.value = req_val.value;
                        break 'outer;
                    }
                }
            }

            'outer: for req_val in request.config.ints {
                for val in &mut config.ints {
                    if req_val.name == val.name && val.value != req_val.value {
                        // TODO(lucasw) also need to clip to min max values
                        tracing::info!(
                            "set '{}' value '{}' to '{}'",
                            val.name,
                            val.value,
                            req_val.value
                        );
                        // TODO(lucasw) clip to min/max
                        val.value = req_val.value;
                        break 'outer;
                    }
                }
            }

            'outer: for req_val in request.config.strs {
                for val in &mut config.strs {
                    if req_val.name == val.name && val.value != req_val.value {
                        // TODO(lucasw) also need to clip to min max values
                        tracing::info!(
                            "set '{}' value '{}' to '{}'",
                            val.name,
                            val.value,
                            req_val.value
                        );
                        val.value = req_val.value;
                        break 'outer;
                    }
                }
            }

            'outer: for req_val in request.config.doubles {
                for val in &mut config.doubles {
                    if req_val.name == val.name && val.value != req_val.value {
                        // TODO(lucasw) also need to clip to min max values
                        tracing::info!(
                            "set '{}' value '{}' to '{}'",
                            val.name,
                            val.value,
                            req_val.value
                        );
                        // TODO(lucasw) clip to min/max
                        val.value = req_val.value;
                        break 'outer;
                    }
                }
            }

            // this panics, also can't do regular send because that requires async
            // let rv = update_sender.blocking_send(true);
            // tracing::info!("update sender: {rv:?}");
            *do_update_pub_copy.lock().unwrap() = true;
            Ok(dynamic_reconfigure::ReconfigureResponse {
                config: config.clone(),
            })
        };

        let full_server_name = get_server_name(full_node_name);
        let server_handle = nh
            .advertise_service::<dynamic_reconfigure::Reconfigure, _>(&full_server_name, server_fn)
            .await
            .expect("can't connect to {full_server_name}");
        tracing::info!("serving dynamic reconfigure server on {full_server_name}");

        let dr = Self {
            config_description,
            config_state,
            update_pub,
            description_pub,
            _server_handle: server_handle,
            // update_receiver,
            do_update_pub,
        };

        Ok(dr)
    }

    pub fn add_bool_param(&mut self, name: &str, value: bool, description: &str) {
        let param = dynamic_reconfigure::BoolParameter {
            name: name.to_string(),
            value,
        };
        let min = dynamic_reconfigure::BoolParameter {
            name: name.to_string(),
            value: false,
        };
        let max = dynamic_reconfigure::BoolParameter {
            name: name.to_string(),
            value,
        };

        let param_description = dynamic_reconfigure::ParamDescription {
            name: param.name.clone(),
            r#type: "bool".to_string(),
            level: 1,
            description: description.to_string(),
            edit_method: "".to_string(),
        };

        // TODO(lucasw) assume only one group, all that is supported currently
        self.config_description.groups[0]
            .parameters
            .push(param_description);

        // Don't need a max string but it's there anyhow, assume the min/max values don't matter
        self.config_description.min.bools.push(min);
        self.config_description.max.bools.push(max);
        self.config_description.dflt.bools.push(param.clone());

        self.config_state.lock().unwrap().bools.push(param);
    }

    fn add_int_param_inner(
        &mut self,
        name: &str,
        value: i32,
        min: i32,
        max: i32,
        description: &str,
        edit_method: String,
    ) {
        let param = dynamic_reconfigure::IntParameter {
            name: name.to_string(),
            value,
        };
        let min = dynamic_reconfigure::IntParameter {
            name: name.to_string(),
            value: min,
        };
        let max = dynamic_reconfigure::IntParameter {
            name: name.to_string(),
            value: max,
        };

        let param_description = dynamic_reconfigure::ParamDescription {
            name: param.name.clone(),
            r#type: "int".to_string(),
            level: 1,
            description: description.to_string(),
            edit_method,
        };

        // TODO(lucasw) assume only one group, all that is supported currently
        self.config_description.groups[0]
            .parameters
            .push(param_description);

        // Don't need a max string but it's there anyhow, assume the min/max values don't matter
        self.config_description.min.ints.push(min);
        self.config_description.max.ints.push(max);
        self.config_description.dflt.ints.push(param.clone());

        self.config_state.lock().unwrap().ints.push(param);
    }

    pub fn add_int_param(&mut self, name: &str, value: i32, min: i32, max: i32, description: &str) {
        self.add_int_param_inner(name, value, min, max, description, "".to_string());
    }

    pub fn add_str_param(&mut self, name: &str, value: &str, description: &str) {
        let param = dynamic_reconfigure::StrParameter {
            name: name.to_string(),
            value: value.to_string(),
        };

        let param_description = dynamic_reconfigure::ParamDescription {
            name: param.name.clone(),
            r#type: "str".to_string(),
            level: 1,
            description: description.to_string(),
            edit_method: "".to_string(),
        };

        // TODO(lucasw) assume only one group, all that is supported currently
        self.config_description.groups[0]
            .parameters
            .push(param_description);

        // Don't need a max string but it's there anyhow, assume the min/max values don't matter
        self.config_description.min.strs.push(param.clone());
        self.config_description.max.strs.push(param.clone());
        self.config_description.dflt.strs.push(param.clone());

        self.config_state.lock().unwrap().strs.push(param);
    }

    pub fn add_double_param(
        &mut self,
        name: &str,
        value: f64,
        min: f64,
        max: f64,
        description: &str,
    ) {
        let param = dynamic_reconfigure::DoubleParameter {
            name: name.to_string(),
            value,
        };
        let min = dynamic_reconfigure::DoubleParameter {
            name: name.to_string(),
            value: min,
        };
        let max = dynamic_reconfigure::DoubleParameter {
            name: name.to_string(),
            value: max,
        };

        let param_description = dynamic_reconfigure::ParamDescription {
            name: param.name.clone(),
            r#type: "double".to_string(),
            level: 1,
            description: description.to_string(),
            edit_method: "".to_string(),
        };

        // TODO(lucasw) assume only one group, all that is supported currently
        self.config_description.groups[0]
            .parameters
            .push(param_description);

        // Don't need a max string but it's there anyhow, assume the min/max values don't matter
        self.config_description.min.doubles.push(min);
        self.config_description.max.doubles.push(max);
        self.config_description.dflt.doubles.push(param.clone());

        self.config_state.lock().unwrap().doubles.push(param);
    }

    pub fn add_enum_param(
        &mut self,
        name: &str,
        value: i32,
        items: Vec<(&str, &str)>,
        enum_description: &str,
    ) {
        // Generate a list of strings to put into edit_method
        //    "{'enum': [
        //    {'name': 'manual', 'type': 'int', 'value': 0, 'srcline': 139, 'srcfile': 'foo.rs', 'description': 'bar', 'ctype': 'int', 'cconsttype': 'const int'},
        //    ...
        //    ], 'enum_description': 'my enum description'}"

        let num = items.len();
        if num == 0 {
            // TODO(lucasw) return error
            tracing::warn!("no items");
            return;
        }
        let max = (num - 1) as i32;

        // TODO(lucasw) could use a json (or yaml) library but just do the above manually for now
        let mut edit_method = "{'enum': [ ".to_string();
        for (index, (name, description)) in items.iter().enumerate() {
            if index > 0 {
                edit_method += ", ";
            }
            let item = format!("{{'name': '{name}', 'type': 'int', 'value': {index}, 'srcline': 0, 'srcfile': 'tbd', 'description': '{description}', 'ctype': 'int', 'cconsttype': 'const int'}}");
            edit_method += &item;
        }
        edit_method += "], 'enum_description': 'my enum description'}";

        self.add_int_param_inner(name, value, 0, max, enum_description, edit_method);
    }

    pub fn init(&self) {
        let _ = self.description_pub.publish(&self.config_description);
    }

    pub fn update(&self) {
        let mut do_update_pub = self.do_update_pub.lock().unwrap();
        if *do_update_pub {
            *do_update_pub = false;
            match self.config_state.lock() {
                Ok(config_state) => {
                    let _ = self.update_pub.publish(&config_state.clone());
                    // let _ = self.description_pub.publish(&config_description);
                }
                Err(err) => {
                    // TODO(lucasw) should return this
                    tracing::warn!("{err:?}");
                }
            }
        }
    }
}
