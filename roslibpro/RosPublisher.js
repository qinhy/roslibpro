import { RosAdvance } from "./RosAdvance.js";

export class RosPublisher extends RosAdvance {
    createTopic() {
        this.pub = this.topic;
    }
    publish(message) {
        this.pub.publish(new ROSLIB.Message(message));
    }
}
