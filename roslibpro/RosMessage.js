
import { RosPublisher } from "./RosPublisher.js";
import { RosService } from "./RosService.js";
import { RosSubscriber } from "./RosSubscriber.js";
import { SingletonKeyValueStorage } from "./Storages.js";
import { EventDispatcherController } from "./EventDispatcher.js";

// # ROS2 components : pub/sub, service

// # class == Namespace Path
// #   variable == Final Endpoint Namespace Path
// #   function starts with "_srv_" == Service
// #   function starts with "_act_" == Action

// # streaming direction => input or output
// #    output : Pub a topic
// #    output == variable == Pub a topic

// #    input  : The instance try to Sub a topic, passive  (try Sub)
// #    input == try Sub == {_}variable ({__}variable is private)

/////////// rules

// /app/app_state -> /app/_app_state
const RosPathAddUnderscore = (path)=>{
    const p = path.split("/");
    p[p.length-1] = "_"+p[p.length-1];
    return p.join("/");
}
// /app/_app_state -> /app/app_state
const RosPathDelUnderscore = (path)=>{
    const p = path.split("/");
    p[p.length-1] = p[p.length-1].slice(1);
    return p.join("/");

}
// /app/app_state -> return root_obj.app.app_state
const RosPathToObjParam = (path,root_obj)=>{
    let param = root_obj;
    for (const param_name of path.split('/')) {
        if (param_name.length === 0) continue;
        param = param[param_name]?param[param_name]:param["_"+param_name];
    }
    return param;
}

const isPrivateParam = (paramName="",param=null)=>{
    const last_paramName = paramName.split("/").pop()
    return last_paramName.startsWith('__') || paramName.includes('/__');
}

const isSubParam = (paramName="",param=null) => {
    const last_paramName = paramName.split("/").pop()
    return last_paramName.startsWith('_') || paramName.includes('/_');
}

const isFunction = (paramName=null,param=null) => {
    return typeof param === 'function';
}

const isPrimitiveParam = (paramName=null,param=null) => {
    if(param === null)return true;
    if(isFunction(param))return true;
    if(param.hasOwnProperty('__empty_ros_message'))return true;
    if(!param.hasOwnProperty('___pubsub_ros_model'))return true;
    return false;
}

const isPubModel = (paramName=null,param=null) => {
    return param.___pubsub_ros_model == 'pub';
}

const filterObject = (data,compare=(key)=>{return true}) => {    
    const tmp_data = {};
    for (const key in data) {
        if (compare(key)) {
            tmp_data[key] = data[key];
        }
    }
    return tmp_data;
}

//////////////////////////////////////// rules end

class RosAbstractModel {
    constructor(empty = false) {
        // super();
        this.___pubsub_ros_model = 'pub';
        if (empty) return;

        // Publishers
        // this.app_state = null//std_msgs.msg.Int32();

        // Subscribers
        // this._operation_cmd = null//std_msgs.msg.Float32MultiArray();
    }

    // service topic with "_srv_" , action topic with "_act_" ,
    // _srv_someservice(){}


    _is_private(param_name) {return isPrivateParam(param_name)};
    _is_sub(param_name) {return isSubParam(param_name)};
    _is_function(param) {return isFunction("",param)};
    _is_primitive(param) {return isPrimitiveParam("",param)};
    _is_pub_model() {return isPubModel("",this)};

    _get_pubs() {return this._dfs(this).filter(([is_f, n, v]) => !is_f && !this._is_sub(n));}
    _get_subs() {return this._dfs(this).filter(([is_f, n, v]) => !is_f && this._is_sub(n));}
    _get_srvs() {return this._dfs(this).filter(([isFunc, n, v]) => isFunc);}

    _get_pubs_path() {
        const name = this.constructor.name;
        return this._get_pubs().map(([isFunc, n, v]) => `${name}/${n}  :  ${v.getTopicType()}`);
    }

    _get_subs_path() {
        const name = this.constructor.name;
        return this._get_subs().map(([isFunc, n, v]) => `${name}/${n}  :  ${v.getTopicType()}`);
    }

    _get_srvs_path() {
        const name = this.constructor.name;
        return this._get_srvs().map(([isFunc, n, v]) => `${name}/${n}  :  [TODO]service type`);
    }

    _dfs(param, path = '', isFunc = false, onlyFunc = false) {
        const paths = [];

        if (this._is_primitive(param)) {
            paths.push([this._is_function(param), path, param]);
        } else {
            for (const [key, value] of Object.entries(param)) {
                if (this._is_private(key)) continue;
                const isFunction = this._is_function(value);
                if (isFunction && (!key.startsWith('_srv_') || !key.startsWith('_act_'))) continue;
                const newPath = `${path}/${key}`;
                paths.push(...this._dfs(value, newPath, isFunction, onlyFunc));
            }
        }
        return paths;
    }

    _pubs() {
        const menbers = {};
        for (const key in this) {
            if (!this[key].hasOwnProperty('___pubsub_ros_model') && this._is_primitive(key) && !this._is_sub(key) && !this._is_private(key)) {
                menbers[key] = this[key];
            }
        }
        return menbers;
    }
    _subs() {
        const menbers = {};
        for (const key in this) {
            if (!this[key].hasOwnProperty('___pubsub_ros_model') && this._is_primitive(key) && this._is_sub(key) && !this._is_private(key)) {
                menbers[key] = this[key];
            }
        }
        return menbers;
    }
    _to_subs(obj) {
        obj = { ...obj };
        for (const key in obj) {
            if (!key.startsWith('_')) {
                const newKey = `_${key}`; // Add "_" to the beginning
                obj[newKey] = obj[key];   // Assign the value to the new key
                delete obj[key];          // Delete the old key
            }
        }
        return obj;
    }
    _to_pubs(obj) {
        obj = { ...obj };
        for (const key in obj) {
            if (key.startsWith('_')) {
                const newKey = key.slice(1); // Remove the first character
                obj[newKey] = obj[key];      // Assign the value to the new key
                delete obj[key];             // Delete the old key
            }
        }
        return obj;
    }

    _toPubModel() {
        var res = this._is_pub_model() ? this : this._switchModel();
        res.___pubsub_ros_model = 'pub';
        return res;
    }

    _toSubModel() {
        var res = this._is_pub_model() ? this._switchModel() : this;
        res.___pubsub_ros_model = 'sub';
        return res;
    }

    _switchModel() {
        const instance = new this.constructor(true);
        const ispub = this._is_pub_model();
        for (const key in this) {
            if (!this._is_primitive(this[key])) {
                if (this[key]._is_pub_model()) {
                    // instance[`_${key}`] = this[key]._toSubModel();
                    instance[key] = this[key]._toSubModel();
                }
                else {
                    // instance[key.slice(1)] = this[key]._toPubModel();
                    instance[key] = this[key]._toPubModel();
                }
            }
        }
        Object.assign(instance, this._to_subs(this._pubs()));
        Object.assign(instance, this._to_pubs(this._subs()));
        instance.___pubsub_ros_model = ispub ? 'sub' : 'pub';
        return instance
    }

    _randomSetMembers() {
        for (const key in this) {
            if (this._is_private(key)) continue;
            if (this[key].hasOwnProperty('__empty_ros_message')) {
                this[key].randomSetMembers();
            }
            else {
                this[key]._randomSetMembers();
            }
        }
    }

}


/// ros messages 
class RosMessageBase extends EventDispatcherController {
    constructor(empty = true) {
        super();
        this.__empty_ros_message = empty;
        this.__topic_name = null;
        this.__pubsub_ros_message = null;
        this.__rosconn = null;
    }

    empty() { return this.__empty_ros_message; }

    toRosBridgeFormat() {
        return filterObject(this,(key)=> this.hasOwnProperty(key) && !key.startsWith('__'));
    }

    fromRosBridgeFormat(data) {
        Object.assign(this, filterObject(data,(key)=> this.hasOwnProperty(key) && !key.startsWith('_')) );
        return this;
    }

    buildRosConn(rosip, topic_name, conn_type = "sub", rate=10) {

        if(conn_type=="sub"){
            this.__topic_name = RosPathDelUnderscore(topic_name);
        }
        else{
            this.__topic_name = topic_name;
        }

        const _this = this;
        this.__rosconn = new ({ 'pub': RosPublisher, 'sub': RosSubscriber, 'srv': RosService
        }[conn_type])(rosip, this.__topic_name, this.getTopicType(), rate);
        console.log(rosip, this.__topic_name, this.getTopicType(), rate);
        this.__rosconn.connectROS({
            onError: (error) => { this.__pubsub_ros_message = null; this.destroyRosConn();},
            onConnection: () => { 
                this.__pubsub_ros_message = conn_type;
                if (conn_type = "sub") {
                    this.__rosconn.subscribe = (msg) => {
                        _this.fromRosBridgeFormat(msg);
                        _this.dispatch('subscribe', _this);
                    }
                }
            },
            onClose: () => { this.destroyRosConn(); }
        }); 
    }

    destroyRosConn() {
        this.__rosconn.destroy();
        this.__rosconn = null;
    }

    sub(sub_msg_callbak = (msg) => { }) {
        const uuid = this.set_event('subscribe', sub_msg_callbak);
        return uuid;
    }

    pub(data=null) { 
        if(data)this.fromRosBridgeFormat(data);
        this.__rosconn.publish(this.toRosBridgeFormat()); 
    };

    getTopicType() {
        return this.constructor.name.replace(/__/g, '/');
    }

    _isPrivate(paramName) {return isPrivateParam(paramName)}

    randomSetMembers() {
        const dfsRandomSet = (obj) => {
            for (let key in obj) {
                if (!this._isPrivate(key) && typeof obj[key] === 'object' && obj[key] !== null) {
                    // Recursively set values for nested objects
                    dfsRandomSet(obj[key]);
                } else if (typeof obj[key] === 'number') {
                    // Set a random number for numeric properties
                    obj[key] = Math.random() * 10; // Adjust range as needed
                } else if (typeof obj[key] === 'string') {
                    // Set a random string for string properties
                    obj[key] = Math.random().toString(36).substring(2, 8); // Generates a random 6-character string
                } else if (typeof obj[key] === 'boolean') {
                    // Set a random boolean value
                    obj[key] = Math.random() < 0.5; // Randomly sets to true or false
                }
            }
        };
        dfsRandomSet(this);
    }
}

class nav_msgs__msg__Odometry extends RosMessageBase {
    constructor() {
        super();
        this.header = { seq: 0, time: "20241010T00:00:00", frame_id: "null" };
        this.child_frame_id = "null";
        this.pose = { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } };
        this.twist = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
    }
}

class sensor_msgs__msg__JointState extends RosMessageBase {
    constructor() {
        super();
        this.header = { seq: 0, time: "20241010T00:00:00", frame_id: "null" };
        this.name = [];
        this.position = [];
        this.velocity = [];
        this.effort = [];
    }
}

class sensor_msgs__msg__PointCloud2 extends RosMessageBase {
    constructor() {
        super();
        this.header = {
            stamp: new Date(),  // Timestamp
            frame_id: ''        // Reference frame
        };
        this.height = 0;
        this.width = 0;
        this.fields = [];     // Array of field structures
        this.is_bigendian = false;
        this.point_step = 0;
        this.row_step = 0;
        this.data = [];       // Actual point data, typically an array of bytes
        this.is_dense = false;
    }
}


class std_msgs__msg__Int8MultiArray extends RosMessageBase {
    constructor() {
        super();
        this.layout = { dim: [], data_offset: 0 };
        this.data = [];
    }
}

class std_msgs__msg__Float32MultiArray extends RosMessageBase {
    constructor() {
        super();
        this.layout = {
            dim: [], // array of dimensions, each with label, size, and stride
            data_offset: 0 // offset for the data in the array
        };
        this.data = []; // array of float32 values
    }
}

class rdt_interfaces__msg__ApplicationState extends RosMessageBase {
    constructor() {
        super();
        this.state = '';
        this.details = '';
    }
}

class visualization_msgs__msg__MarkerArray extends RosMessageBase {
    constructor() {
        super();
        this.markers = [];
    }

    fromRosBridgeFormat(data) {
        const instance = new this.constructor();
        instance.markers = data.markers.map(markerData => visualization_msgs__msg__Marker().fromRosBridgeFormat(markerData));
        return instance;
    }
}

class visualization_msgs__msg__Marker extends RosMessageBase {
    constructor() {
        super();
        this.header = { seq: 0, time: "20241010T00:00:00", frame_id: "null" };
        this.ns = '';
        this.id = 0;
        this.type = 0;
        this.action = 0;
        this.pose = { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } };
        this.scale = { x: 1, y: 1, z: 1 };
        this.color = { r: 1.0, g: 1.0, b: 1.0, a: 1.0 };
        this.lifetime = 0;
        this.points = [];
        this.text = '';
    }
}

class nav_msgs__msg__Path extends RosMessageBase {
    constructor() {
        super();
        this.header = { seq: 0, time: "20241010T00:00:00", frame_id: "null" };
        this.poses = [];
    }

    fromRosBridgeFormat(data) {
        const instance = new this.constructor();
        instance.header = data.header;
        instance.poses = data.poses.map(poseData => this.fromRosBridgeFormat(poseData));
        return instance;
    }
}

class geometry_msgs__msg__PoseStamped extends RosMessageBase {
    constructor() {
        super();
        this.header = { seq: 0, time: "20241010T00:00:00", frame_id: "null" };
        this.pose = { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } };
    }
}

class std_msgs__msg__Bool extends RosMessageBase {
    constructor() {
        super();
        this.data = false;
    }
}

class std_msgs__msg__String extends RosMessageBase {
    constructor(data = '') {
        super();
        this.data = data;
    }
}

class std_msgs__msg__Int32 extends RosMessageBase {
    constructor(data = 0) {
        super();
        this.data = data;
    }
}
class moveit_msgs__msg__PlanningScene extends RosMessageBase {
    constructor() {
        super();
        this.name = '';
        this.robot_state = {
            joint_state: {
                name: [],
                position: [],
                velocity: [],
                effort: []
            },
            multi_dof_joint_state: {
                header: {
                    stamp: new Date(),
                    frame_id: ''
                },
                joint_names: [],
                transforms: [], // Array of transformation data (position + orientation)
                twist: [],
                wrench: []
            }
        };
        this.robot_model_name = '';
        this.fixed_frame_transforms = [];
        this.allowed_collision_matrix = {};
        this.link_padding = [];
        this.link_scale = [];
        this.object_colors = [];
        this.world = {
            collision_objects: [],
            octomap: {}
        };
        this.is_diff = false;
    }
}


const sensor_msgs = { msg: {} };
const rdt_interfaces = { msg: {} };
const visualization_msgs = { msg: {} };
const nav_msgs = { msg: {} };
const geometry_msgs = { msg: {} };
const std_msgs = { msg: {} };
const moveit_msgs = { msg: {} };

nav_msgs.msg.Odometry = nav_msgs__msg__Odometry;
sensor_msgs.msg.JointState = sensor_msgs__msg__JointState;
sensor_msgs.msg.PointCloud2 = sensor_msgs__msg__PointCloud2;
std_msgs.msg.Int8MultiArray = std_msgs__msg__Int8MultiArray;
std_msgs.msg.Float32MultiArray = std_msgs__msg__Float32MultiArray;
rdt_interfaces.msg.ApplicationState = rdt_interfaces__msg__ApplicationState;
visualization_msgs.msg.MarkerArray = visualization_msgs__msg__MarkerArray;
visualization_msgs.msg.Marker = visualization_msgs__msg__Marker;
nav_msgs.msg.Path = nav_msgs__msg__Path;
geometry_msgs.msg.PoseStamped = geometry_msgs__msg__PoseStamped;
std_msgs.msg.Bool = std_msgs__msg__Bool;
std_msgs.msg.String = std_msgs__msg__String;
std_msgs.msg.Int32 = std_msgs__msg__Int32;
moveit_msgs.msg.PlanningScene = moveit_msgs__msg__PlanningScene;


class RosRoot extends RosAbstractModel {
    constructor() {
        super();
    }

    connect(rosip){
        this._get_pubs().forEach(([isFunc,path,obj])=>{
            obj.buildRosConn(rosip,path,'pub');
        })
        this._get_subs().forEach(([isFunc,path,obj])=>{
            obj.buildRosConn(rosip,path,'sub');
        })

    }

    pub_topic(path, ...args) {
        if (this._is_sub(path)) return 'Cannot pub a "try Sub" topic';
        const param = RosPathToObjParam(path,this);
        if (!this._is_function(param) && this._is_primitive(param)) {
            param.pub(...args);
        }
    }

    sub_topic(path, sub_msg_callbak = (msg) => { }) {
        // root.app._operation_cmd.add_sub_listener((msg)=>{console.log(msg)});
        // root.sub_topic("/app/operation_cmd",(msg)=>{console.log(msg)});
        if (this._is_sub(path)) return 'Cannot pub a "try Sub" topic';
        path = RosPathAddUnderscore(path)
        const param = RosPathToObjParam(path,this);
        if (!this._is_function(param) && this._is_primitive(param)) {
            param.sub(sub_msg_callbak);
        }
    }

    echo_topic(path) {
        if (this._is_sub(path)) return 'Cannot echo a "try Sub" topic';
        path = RosPathAddUnderscore(path)
        const param = RosPathToObjParam(path,this);
        if (!this._is_function(param) && this._is_primitive(param)) {
            return param;
        }
        return null;
    }

    call_service(path, ...args) {
        path = RosPathAddUnderscore(path)
        const param = RosPathToObjParam(path,this);
        if (this._is_function(param)) {
            return param(...args);
        }
        return null;
    }

    all_list() {
        let res = '\n----------------------------------------';
        res += '\npub topics:\n';
        res += this._get_pubs_path().map(pub => pub.replace('RosRoot/', '')).join('\n') + '\n';
        res += '\nsub topics:\n';
        res += this._get_subs_path().map(sub => sub.replace('RosRoot/', '')).join('\n') + '\n';
        res += '\nsrv topics:\n';
        res += this._get_srvs_path().map(srv => srv.replace('RosRoot/', '')).join('\n') + '\n';
        res += '----------------------------------------\n';
        return res;
    }

    service_list() {
        let res = '\n----------------------------------------';
        res += '\nsrv topics:\n';
        res += this._get_srvs_path().map(srv => srv.replace('RosRoot/', '')).join('\n') + '\n';
        res += '----------------------------------------\n';
        return res;
    }

    topic_list() {
        let res = '\n----------------------------------------';
        res += '\npub topics:\n';
        res += this._get_pubs_path().map(pub => pub.replace('RosRoot/', '')).join('\n') + '\n';
        res += '\nsub topics:\n';
        res += this._get_subs_path().map(sub => sub.replace('RosRoot/', '')).join('\n') + '\n';
        res += '----------------------------------------\n';
        return res;
    }
}

export { sensor_msgs, moveit_msgs, rdt_interfaces, visualization_msgs, nav_msgs, geometry_msgs, std_msgs, RosAbstractModel, RosRoot }