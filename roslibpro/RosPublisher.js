import { RosAdvance } from "./RosAdvance.js";

export class RosPublisher extends RosAdvance {
    constructor(rosip, topic_name, topic_type, rate = 10) {
        super(rosip, topic_name, topic_type, rate, false);
    }
    createTopic() {
        this.pub = ()=>this.topic();
    }
    publish(message) {
        // console.log(message);
        this.pub().publish(new ROSLIB.Message(message));
    }
}
