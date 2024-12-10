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
    } else if let Ok(value) = raw_value.parse::<i32>() {
        let int_param = dynamic_reconfigure::IntParameter { name, value };
        req.config.ints.push(int_param);
        param_type = ParamType::INT;
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
            // for intv in request.config.ints {
            // }
            'outer: for req_strv in request.config.strs {
                for strv in &mut config.strs {
                    if req_strv.name == strv.name && strv.value != req_strv.value {
                        // TODO(lucasw) also need to clip to min max values
                        tracing::info!(
                            "set '{}' value '{}' to '{}'",
                            strv.name,
                            strv.value,
                            req_strv.value
                        );
                        strv.value = req_strv.value;
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
