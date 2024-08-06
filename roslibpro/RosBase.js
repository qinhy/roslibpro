const RosBaseStateList = ['NULL','Connecting','ConnectDone','Closing','Closed','Failure',]
const RosBaseStates = {
    NULL:'NULL',
    Connecting:'Connecting',
    ConnectDone:'ConnectDone',
    Closing:'Closing',
    Closed:'Closed',
    Failure:'Failure',
}
const RosBaseTransitions = {
    NULL: [RosBaseStates.Connecting,RosBaseStates.Failure],
    Connecting: [RosBaseStates.ConnectDone,RosBaseStates.Failure],
    ConnectDone: [RosBaseStates.Closing,],
    Closing: [RosBaseStates.Closed,RosBaseStates.Failure],
    Closed: [RosBaseStates.NULL],
    Failure: [RosBaseStates.NULL],
}

// model and controller
class RosBase { 
    constructor(rosip, topic_name, topic_type, rate = 10, is_service = false,
                onError=(error) => { },
                onConnection=() => { },
                onClose=() => { }
    ) {
        this.rosip = rosip;
        this.topic_name = topic_name;
        this.topic_type = topic_type;
        this.rate = rate;
        this.self_closed = false;
        this.connected_cnt = 0;
        this.is_service = is_service;
        this.state = null;
        this.onError=onError;
        this.onConnection=onConnection;
        this.onClose=onClose;
        this.getROS = ()=>null;
        this.topic = ()=>null;
        this.setState(StateNULL);
        
    }
    getTransitions(){
        return RosBaseTransitions;
    }
    currentState(){
        return this.state.constructor.name;
    }
    setState(state_class){
        this.state = new state_class(this);
    }
    close() {        
        if(this.topic() && this.topic().unadvertise)this.topic().unadvertise();        
        if (this.getROS() && this.getROS().socket && this.getROS().socket.readyState !== WebSocket.CLOSED) {
            this.getROS().close();
        }
    }

    to_NULL(){          this.state.to_NULL();}
    to_ConnectDone(){   this.state.to_ConnectDone();}
    to_Connecting(){    this.state.to_Connecting();}
    to_Closing(){       this.state.to_Closing();}
    to_Closed(){        this.state.to_Closed();}
    to_Failure(){       this.state.to_Failure();}

    findPath(transitions, startState, endState) {
        let queue = [[startState]];
        let visited = new Set();
        while (queue.length > 0) {
            let path = queue.shift();
            let state = path[path.length - 1];
            if (state === endState) {
                return path;
            }
            if (!visited.has(state)) {
                visited.add(state);
                let nextStates = transitions[state] || [];
                for (let nextState of nextStates) {
                    let newPath = path.slice();
                    newPath.push(nextState);
                    queue.push(newPath);
                }
            }
        }
        return [];
    }

    resumeState(targetState, maxAttempts = 100) {
        console.log(`Set target state: ${targetState} (current is ${this.currentState()})`);
        
        const nextAction = (task, targetState) => {
            let path = task.findPath(task.getTransitions(), task.currentState(), targetState);
            if (path.length <= 1) return null;
            return path[1];
        };

        while (this.currentState() !== targetState) {
            let cls = nextAction(this, targetState);
            if (cls === null) throw new Error('No next action! Unreachable!');
            if (maxAttempts < 0) throw new Error('Over max_attempts!');
            
            console.log(`Current: ${this.currentState()}, try to_${cls}`);
            this[`to_${cls}`]();
            maxAttempts -= 1;
        }
        console.log(`Success to target state: ${this.currentState()}`);
    }

}
class StateBase{
    constructor(controller) {
        this.controller = controller;
    }
    _NULL() {
        this.controller.close();
        this.controller.getROS = ()=>null;
        this.controller.topic = ()=>null;
        this.controller.setState(StateNULL);
    }
    _Failure() {this.controller.setState(StateFailure);}

    _defaultError(to) {
        throw new Error(`Invalid transition from [${this.constructor.name}] -> [${to}]`);
    }
    to_NULL(){          this._defaultError("NULL");}
    to_ConnectDone(){   this._defaultError("ConnectDone");}
    to_Connecting(){    this._defaultError("Connecting");}
    to_Closing(){       this._defaultError("Closing");}
    to_Closed(){        this._defaultError("Closed");}
    to_Failure(){       this._defaultError("Failure");}
}
class StateNULL extends StateBase{
    _transitions = [RosBaseStates.Connecting,RosBaseStates.Failure]
    to_Connecting(){     
        try {
            const ros = new ROSLIB.Ros({url: `ws://${this.controller.rosip}`});
            this.controller.getROS = ()=>ros;
            this.controller.getROS().on("error", error => {
                this.controller.getROS = ()=>null;
                this.controller.onError(error);
                this.controller.to_Failure();
            });

            this.controller.getROS().on("connection", () => {
                this.controller.onConnection();
                this.controller.to_ConnectDone();
            });

            this.controller.getROS().on("close", () => {                
                this.controller.getROS = ()=>null;
                this.controller.onClose();
            });

            const topic = this.controller.is_service
                ? new ROSLIB.Service({
                    ros: this.controller.getROS(),
                    name: this.controller.topic_name,
                    serviceType: this.controller.topic_type
                })
                : new ROSLIB.Topic({
                    ros: this.controller.getROS(),
                    name: this.controller.topic_name,
                    messageType: this.controller.topic_type,
                    rate: this.controller.rate
                });
            this.controller.topic = ()=>topic;
            this.setState(StateConnecting);
        } catch (e) {
            this._Failure();
        }
    }
    to_Failure(){this._Failure()}
}
class StateConnecting extends StateBase{
    _transitions = [RosBaseStates.ConnectDone,RosBaseStates.Failure]
    to_Failure(){this._Failure()}
    to_ConnectDone(){
        if(this.getROS() && this.getROS().socket.readyState === WebSocket.OPEN){
            this.controller.setState(StateConnectDone);
        }
    }
}
class StateConnectDone extends StateBase{
    _transitions = [RosBaseStates.Closing,]
    to_Closing(){
        this.controller.setState(StateClosing);
        this.controller.close();
    }

}
class StateClosing extends StateBase{
    _transitions = [RosBaseStates.Closed,RosBaseStates.Failure]
    to_Failure(){this._Failure()}
    to_Closed(){
        if(this.getROS() || this.getROS().socket.readyState === WebSocket.CLOSED){
            this.controller.setState(StateClosed);
        }
    }
}
class StateClosed extends StateBase{
    _transitions = [RosBaseStates.NULL]
    to_NULL(){this._NULL();}
}
class StateFailure extends StateBase{
    _transitions = [RosBaseStates.NULL]
    to_NULL(){this._NULL();}
}


export class RosBase {
    constructor(rosip, topic_name, topic_type, rate = 10, is_service = false) {
        this.rosip = rosip;
        this.topic_name = topic_name;
        this.topic_type = topic_type;
        this.rate = rate;
        this.self_closed = false;
        this.connected_cnt = 0;
        this.is_service = is_service;
        this.getROS = ()=>null;
        this.topic = ()=>null;
    }

    connectROS(callback = {
        onError: (error) => { },
        onConnection: () => { },
        onClose: () => { }
    }, call_from_outter = true) {
        if (this.isConnectDone() || this.isConnecting() || this.isClosing()) return false;

        if (call_from_outter) {
            this.connected_cnt = 0;
            this.self_closed = false;
        }

        try {
            const ros = new ROSLIB.Ros({
                url: `ws://${this.rosip}`
            });
            this.getROS = ()=>ros;

            this.getROS().on("error", error => {
                
        this.getROS = ()=>null;
                if (call_from_outter) this.connected_cnt++;
                callback.onError(error);
            });

            this.getROS().on("connection", () => {
                this.self_closed = false;
                if (call_from_outter) this.connected_cnt++;
                callback.onConnection();
            });

            this.getROS().on("close", () => {
                
        this.getROS = ()=>null;
                callback.onClose();
            });

            const topic = this.is_service
                ? new ROSLIB.Service({
                    ros: this.getROS(),
                    name: this.topic_name,
                    serviceType: this.topic_type
                })
                : new ROSLIB.Topic({
                    ros: this.getROS(),
                    name: this.topic_name,
                    messageType: this.topic_type,
                    rate: this.rate
                });
            this.topic = ()=>topic;
        } catch (e) {
            return false;
        }
        return true;
    }

    isNULL() {
        return this.getROS() === null || this.getROS().socket === null;
    }

    isConnectDone() {
        return !this.isNULL() && this.getROS().socket.readyState === WebSocket.OPEN;
    }

    isConnecting() {
        return !this.isNULL() && this.getROS().socket.readyState === WebSocket.CONNECTING;
    }

    isClosing() {
        return !this.isNULL() && this.getROS().socket.readyState === WebSocket.CLOSING;
    }

    isClosed() {
        return this.isNULL() || this.getROS().socket.readyState === WebSocket.CLOSED;
    }

    close(is_self_closed = true) {
        this.self_closed = is_self_closed;
        
        if(this.topic() && this.topic().unadvertise)this.topic().unadvertise();
        this.topic = ()=>null;
        
        if (this.getROS() && this.getROS().socket && this.getROS().socket.readyState !== WebSocket.CLOSED) {
            this.getROS().close();
        }
        
        this.getROS = ()=>null;
    }
}
