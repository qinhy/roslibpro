import { RosSubscriber } from "./RosSubscriber.js";
import * as THREE from "../libs/threejs/three.module.js";
import {Spectral_r} from "../libs/js-colormaps/js-colormaps.js";

const PointCloud2Config = {};
// constants used for decoding raw point clouds (not used for decoding compressed)
PointCloud2Config.INT8 = 1;
PointCloud2Config.UINT8 = 2;
PointCloud2Config.INT16 = 3;
PointCloud2Config.UINT16 = 4;
PointCloud2Config.INT32 = 5;
PointCloud2Config.UINT32 = 6;
PointCloud2Config.FLOAT32 = 7;
PointCloud2Config.FLOAT64 = 8;
PointCloud2Config.SIZEOF = {
  [PointCloud2Config.INT8]: 1,
  [PointCloud2Config.UINT8]: 1,
  [PointCloud2Config.INT16]: 2,
  [PointCloud2Config.UINT16]: 2,
  [PointCloud2Config.INT32]: 4,
  [PointCloud2Config.UINT32]: 4,
  [PointCloud2Config.FLOAT32]: 4,
  [PointCloud2Config.FLOAT64]: 8,
}

// advertise this Config to the system

PointCloud2Config.friendlyName = "Point cloud (3D)";
PointCloud2Config.supportedTypes = [
    "sensor_msgs/msg/PointCloud2",
];

export class RosPointCloud2Subscriber extends RosSubscriber{

    subscribe(msg){
        this.points_threejs = null;
        //this.points_threejs will be set at below
        const pointsinfo = this.onPointCloud2Data(msg);
    }

    onPointCloud2Data(msg) {  
      let points = null;
      if(msg.__comp) {
        points = this.decodeAndRenderCompressed(msg);
      } else {
        points = this.decodeAndRenderUncompressed(msg);
      }
      return points;
    }

    _getDataGetter(datatype, view) {
        // given a JS DataView view and a ROS PointField datatype,
        // fetch the getXXX function for that datatype.
        // (needed to bind it back to the view before returning, or else it doesn't work)
        // used for decoding raw point clouds
        switch(datatype) {
        case PointCloud2Config.INT8:
            return view.getInt8.bind(view);
        case PointCloud2Config.UINT8:
            return view.getUInt8.bind(view);
        case PointCloud2Config.INT16:
            return view.getInt16.bind(view);
        case PointCloud2Config.UINT16:
            return view.getUInt16.bind(view);
        case PointCloud2Config.INT32:
            return view.getInt32.bind(view);
        case PointCloud2Config.UINT32:
            return view.getUInt32.bind(view);
        case PointCloud2Config.FLOAT32:
            return view.getFloat32.bind(view);
        case PointCloud2Config.FLOAT64:
            return view.getFloat64.bind(view);
        default:
            return (offset, littleEndian) => {return 0.0};
        }
    }
    _base64decode(base64) {
        var binary_string = window.atob(base64);
        var len = binary_string.length;
        var bytes = new Uint8Array(len);
        for (var i = 0; i < len; i++) {
            bytes[i] = binary_string.charCodeAt(i);
        }
        return bytes.buffer;
    }

    decodeAndRenderCompressed(msg) {
        // decodes a uint16 lossy-compressed point cloud
        // basic explanation of algorithm:
        // - keep only x,y,z fields and throw away the rest
        // - throw away rows containing nans
        // - compress each number into a uint16 from 0 to 65535
        //   where 0 is the minimum value over the whole set and 65535 is the max value over the set
        //   so for example if the x values range from -95 m to 85 m, we encode x into a uint16 where
        //   0 represents -95 and 65535 represents 85
        // - provide the actual bounds (i.e. [-95, 85]) in a separate bounds field so it can be
        //   scaled back correctly by the decompressor (this function)
    
        let bounds = msg._data_uint16.bounds;
        let points_data = this._base64decode(msg._data_uint16.points);
        let points_view = new DataView(points_data);

        let points = new Float32Array(Math.round(points_data.byteLength / 2));

        let xrange = bounds[1] - bounds[0];
        let xmin = bounds[0];
        let yrange = bounds[3] - bounds[2];
        let ymin = bounds[2];
        let zrange = bounds[5] - bounds[4];
        let zmin = bounds[4];

        
        const vertices = [];
        const colors = [];
        
        var npoints = points_data.byteLength/6;
        for(let i=0; i<npoints; i++) {
        var vertex = new THREE.Vector3();
        var color = new THREE.Color();
        let offset = i * 6;
        vertex.x = points[3*i] = (points_view.getUint16(offset, true) / 65535) * xrange + xmin;
        vertex.y = points[3*i+1] = (points_view.getUint16(offset+2, true) / 65535) * yrange + ymin;
        vertex.z = points[3*i+2] = (points_view.getUint16(offset+4, true) / 65535) * zrange + zmin;
        var r = (vertex.z - (-0.5))/2.5;//-0.5~2.0m
        r = r > 1 ? 1:r
        r = r < 0 ? 0:r;
        var rgb = Spectral_r(r);
        color.setRGB(rgb[0]/255.0,rgb[1]/255.0,rgb[2]/255.0);
        vertices.push(vertex.x,vertex.y,vertex.z);
        colors.push(color.r,color.g,color.b)
        }
        var geometry = new THREE.BufferGeometry();    
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
        geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
        geometry.computeBoundingSphere();
        this.points_threejs = new THREE.Points(geometry,  new THREE.PointsMaterial({size: 0.1, vertexColors:true}));
        
        return {type: "points", data: npoints, zmin: zmin, zmax: zmin + zrange}

        // this.draw([
        //   {type: "path", data: [0, 0, 0, 1], color: "#00f060", lineWidth: 2},
        //   {type: "path", data: [0, 0, 1, 0], color: "#f06060", lineWidth: 2},
        //   {type: "points", data: points, zmin: zmin, zmax: zmin + zrange},
        // ]);
}

    decodeAndRenderUncompressed(msg) {
        // decode an uncompressed pointcloud.
        // expects "data" field to be base64 encoded raw bytes but otherwise follows ROS standards

        let fields = {};
        let actualRecordSize = 0;

        msg.fields.forEach((field) => {
            fields[field.name] = field;
            if(!field.datatype in PointCloud2Viewer.SIZEOF) {
            this.error("Invalid PointCloud2 message: field " + field + " has invalid datatype = " + String(field.datatype));
            return;
            }
            actualRecordSize += PointCloud2Viewer.SIZEOF[field.datatype];
        });

        if(!("x" in fields) || !("y") in fields) {
            this.error("Cannot display PointCloud2 message: Must have at least 'x' and 'y' fields or I don't know how to display it.");
        }

        let data = this._base64decode(msg.data);

        if(!(msg.point_step * msg.width * msg.height === data.byteLength)) {
            this.error("Invalid PointCloud2: failed assertion: point_step * width * height === data.length");
            return;
        }

        let points = new Float32Array(Math.round(data.byteLength / msg.point_step * 3));
        let view = new DataView(data);
        let littleEndian = !msg.is_bigendian;

        // cache these into variables to avoid hitting hash repeatedly
        let xOffset = fields["x"].offset;
        let xDataGetter = this._getDataGetter(fields["x"].datatype, view);
        let yOffset = fields["y"].offset;
        let yDataGetter = this._getDataGetter(fields["y"].datatype, view);
        let zOffset = -1;
        let zDataGetter = null;
        if("z" in fields) {
            zOffset = fields["z"].offset;
            zDataGetter = this._getDataGetter(fields["z"].datatype, view);
        }

        const vertices = [];
        const colors = [];
        var npoints = data.byteLength/msg.point_step-1;
        for(let i=0; i<npoints; i++) {
            var vertex = new THREE.Vector3();
            var color = new THREE.Color();
            let offset = i * msg.point_step;
            vertex.x = points[3*i] = xDataGetter(offset + xOffset, littleEndian); // x
            vertex.y = points[3*i+1] = yDataGetter(offset + yOffset, littleEndian); // y
            vertex.z = points[3*i+2] = zDataGetter(offset + zOffset, littleEndian); // y
            var r = (vertex.z - (-0.5))/2.5;//-0.5~2.0m
            r = r > 1 ? 1:r;
            r = r < 0 ? 0:r;
            var rgb = Spectral_r(r);
            color.setRGB(rgb[0]/255.0,rgb[1]/255.0,rgb[2]/255.0);
            vertices.push(vertex.x,vertex.y,vertex.z);
            colors.push(color.r,color.g,color.b)
            
            // geometry.vertices.push(vertex);
            // geometry.colors.push(color);
        }
        var geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
        geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
        geometry.computeBoundingSphere();
        this.points_threejs = new THREE.Points(geometry,  new THREE.PointsMaterial({size: 0.1,vertexColors: true}));
        return {type: "points", data: npoints}

    // this.draw([
    //   {type: "path", data: [0, 0, 0, 1], color: "#00f060", lineWidth: 2},
    //   {type: "path", data: [0, 0, 1, 0], color: "#f06060", lineWidth: 2},
    //   {type: "points", data: points, zmin: -2.0, zmax: 2.0},
    // ]);
    }
}