import ROSLIB from 'roslib';
import { RosAdvance } from "./RosAdvance.js";
export class RosService extends RosAdvance {
    constructor(rosip, topic_name, topic_type, rate = 10) {
        super(rosip, topic_name, topic_type, rate, true);
    }
    callService(param = {}, timeout = 1000) {
        if(!this.topic){
            throw Error('You must call connectROS(...) at first!')
        }
        this.client = this.topic;
        const _this = this;
        this.service_response = null;
        return new Promise((resolve, reject) => {
            try {
                _this.client.callService(
                    new ROSLIB.ServiceRequest(param), 
                    function (result) {
                        _this.service_response = result;
                        resolve(result);
                });
            } catch (err) {
                reject(err);
            }
        });

    }
}
