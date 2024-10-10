
class RosAbstractModel {
    constructor() {
        this.___pubsub = 'pub';

        // Publishers
        // BH -> UI, 1000ms, std_msgs/msg/Int32
        // this.app_state = null//std_msgs.msg.Int32();

        // Subscribers
        // UI -> BH, XXms, std_msgs/msg/Float32MultiArray
        // this._operation_cmd = null//std_msgs.msg.Float32MultiArray();
    }
    _isPubModel() {
        return this.___pubsub == 'pub';
    }

    _isPrivate(paramName) {
        return paramName.includes('__') || paramName.includes('/__');
    }

    _isSub(paramName) {
        return paramName.startsWith('_') || paramName.includes('/_');
    }

    _isPrimitive(obj) {
        return obj.hasOwnProperty('___pubsub');
    }

    _pubs() {
        const menbers = {};
        for (const key in this) {
            if (this.hasOwnProperty(key) && !this._isSub(key) && !this._isPrivate(key)) {
                menbers[key] = this[key];
            }
        }
        return menbers;
    }
    _subs() {
        const menbers = {};
        for (const key in this) {
            if (this.hasOwnProperty(key) && this._isSub(key) && !this._isPrivate(key)) {
                menbers[key] = this[key];
            }
        }
        return menbers;
    }
    _to_subs(obj) {
        obj = {...obj};
        for (const key in obj) {
            if (obj.hasOwnProperty(key) && !key.startsWith('_')) {
                const newKey = `_${key}`; // Add "_" to the beginning
                obj[newKey] = obj[key];   // Assign the value to the new key
                delete obj[key];          // Delete the old key
            }
        }
        return obj;
    }
    _to_pubs(obj) {
        obj = {...obj};
        for (const key in obj) {
            if (obj.hasOwnProperty(key) && key.startsWith('_')) {
                const newKey = key.slice(1); // Remove the first character
                obj[newKey] = obj[key];      // Assign the value to the new key
                delete obj[key];             // Delete the old key
            }
        }
        return obj;
    }
    _remove_all(obj) {
        for (const key in obj) {
            if (obj.hasOwnProperty(key)) {
                delete obj[key]; // Deletes each property
            }
        }
    }
    _switchModel() {
        const instance = new this.constructor();
        this._remove_all(instance);
        for (const key in this) {
            if (!this._isPrimitive(this[key])) {
                if(this[key]._isPubModel()){
                    instance[`_${key}`] = this[key]._switchModel();
                }
                else{
                    instance[key.slice(1)] = this[key]._switchModel();
                }                
            }
        }
        Object.assign(instance, this._to_subs(this._pubs()));
        Object.assign(instance, this._to_pubs(this._subs()));
        instance.___pubsub = instance.___pubsub == 'pub'?'sub':'pub';
        return instance
    }
}


/// ros messages 
class RosMessageBase {
    constructor() {
        this.__empty = true;
    }
    empty() { return this.__empty; }
    toRosFormat() {
        const publicData = {};
        for (const key in this) {
            if (this.hasOwnProperty(key) && !key.startsWith('__')) {
                publicData[key] = this[key];
            }
        }
        return publicData;
    }
    static fromRosBridgeFormat(data) {
        const instance = new this();
        obj = Object.assign(instance, data);
        this.__empty = false;
        return obj;
    }
    static getTopicType() {
        return this.name.replace(/__/g, '/');
    }
}

class nav_msgs__msg__Odometry extends RosMessageBase {
    constructor() {
        super();
        this.header = null;
        this.childFrameId = null;
        this.pose = { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } };
        this.twist = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
    }
}

class sensor_msgs__msg__JointState extends RosMessageBase {
    constructor() {
        super();
        this.header = null;
        this.name = [];
        this.position = [];
        this.velocity = [];
        this.effort = [];
    }
}

class std_msgs__msg__Int8MultiArray extends RosMessageBase {
    constructor() {
        super();
        this.layout = { dim: [], dataOffset: 0 };
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
        this.state = null;
        this.details = '';
    }
}

class visualization_msgs__msg__MarkerArray extends RosMessageBase {
    constructor() {
        super();
        this.markers = [];
    }

    static fromRosBridgeFormat(data) {
        const instance = new this();
        instance.markers = data.markers.map(markerData => visualization_msgs__msg__Marker.fromRosBridgeFormat(markerData));
        return instance;
    }
}

class visualization_msgs__msg__Marker extends RosMessageBase {
    constructor() {
        super();
        this.header = null;
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
        this.header = null;
        this.poses = [];
    }

    static fromRosBridgeFormat(data) {
        const instance = new this();
        instance.header = data.header;
        instance.poses = data.poses.map(poseData => geometry_msgs__msg__PoseStamped.fromRosBridgeFormat(poseData));
        return instance;
    }
}

class geometry_msgs__msg__PoseStamped extends RosMessageBase {
    constructor() {
        super();
        this.header = null;
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

const sensor_msgs = { msg: {} };
const rdt_interfaces = { msg: {} };
const visualization_msgs = { msg: {} };
const nav_msgs = { msg: {} };
const geometry_msgs = { msg: {} };
const std_msgs = { msg: {} };

nav_msgs.msg.Odometry = nav_msgs__msg__Odometry;
sensor_msgs.msg.JointState = sensor_msgs__msg__JointState;
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


// tests 

// class VisModel extends RosAbstractModel{
//     constructor() {
//         super();
//         // Publishers
//         // BH -> UI, 1000ms, std_msgs/msg/Int32
//         this.app_state = new visualization_msgs.msg.Marker();

//     }
// }
// class AppModel extends RosAbstractModel{
//     constructor() {
//         super();
//         // Publishers
//         // BH -> UI, 1000ms, std_msgs/msg/Int32
//         this.app_state = new std_msgs.msg.Int32();

//         // Subscribers
//         // UI -> BH, XXms, std_msgs/msg/Float32MultiArray
//         this._operation_cmd = new std_msgs.msg.Float32MultiArray();
//         this._vis = new VisModel();

//     }
// }

// app = new AppModel();

export { sensor_msgs, rdt_interfaces, visualization_msgs, nav_msgs, geometry_msgs, std_msgs , RosAbstractModel}