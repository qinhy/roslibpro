
class RosBaseState {
    static States = {
        NULL: 'NULL',
        ConnectDone: 'ConnectDone',
        Connecting: 'Connecting',
        Closing: 'Closing',
        Closed: 'Closed',
        Failure: 'Failure'
    };

    static transitions = {
        [RosBaseState.States.NULL]: [RosBaseState.States.Connecting, RosBaseState.States.Failure],
        [RosBaseState.States.Connecting]: [RosBaseState.States.ConnectDone, RosBaseState.States.Failure],
        [RosBaseState.States.ConnectDone]: [RosBaseState.States.Closing, RosBaseState.States.Failure],
        [RosBaseState.States.Closing]: [RosBaseState.States.Closed, RosBaseState.States.Failure],
        [RosBaseState.States.Closed]: [RosBaseState.States.NULL],
        [RosBaseState.States.Failure]: [RosBaseState.States.NULL]
    };

    constructor(controller) {
        this.controller = controller;
        this.state = RosBaseState.States.NULL;
    }

    toFailure() {
        this.state = RosBaseState.States.Failure;
    }

    handleErrors(func_name,func) {
        return (...args) => {
            const validTransitions = RosBaseState.transitions[this.state];
            const targetTransition = func_name.replace('to', '');
            if (!validTransitions.includes(targetTransition)) {
                throw new Error(`Invalid transition from [${this.state}] -> [${targetTransition}]`);
            }
            try {
                return func.apply(this, args);
            } catch (e) {
                console.error(`[${this.constructor.name}]: ${e}`);
                this.toFailure();
            }
        };
    }

    toNULL = this.handleErrors('toNULL',function () {
        this.controller.getROS = () => null;
        this.state = RosBaseState.States.NULL;
    });

    toConnecting = this.handleErrors('toConnecting',function () {
        this.state = RosBaseState.States.Connecting;

        return new Promise((resolve, reject) => {
                        const ros = new ROSLIB.Ros({
                url: `ws://${this.controller.rosip}`
            });
            this.controller.getROS = () => ros;

            this.controller.getROS().on("error", error => {
                this.controller.getROS = () => null;
                if (this.controller.call_from_outter) this.controller.connected_cnt++;
                this.toFailure();
                reject(error);
            });

            this.controller.getROS().on("connection", () => {
                this.self_closed = false;
                if (this.controller.call_from_outter) this.controller.connected_cnt++;
                this.toConnectDone();
                resolve(this.controller);
            });

            this.controller.getROS().on("close", () => {
                // auto close if on error
                if(this.state==RosBaseState.States.ConnectDone){                    
                    this.toClosing();
                    this.toClosed();             
                }
                resolve(this.controller);
            });
        });
        
    });

    toConnectDone = this.handleErrors('toConnectDone',function () {
        if(this.controller.isConnectDone()){
            this.state = RosBaseState.States.ConnectDone;
        }
    });

    toClosing = this.handleErrors('toClosing',function () {
        this.state = RosBaseState.States.Closing;
        this.controller.close();
        this.toClosed();
    });

    toClosed = this.handleErrors('toClosed',function () {
        this.state = RosBaseState.States.Closed;
    });

    findPath(transitions, startState, endState) {
        const queue = [[startState]];
        const visited = new Set();

        while (queue.length > 0) {
            const path = queue.shift();
            const state = path[path.length - 1];
            if (state === endState) {
                return path;
            }
            if (!visited.has(state)) {
                visited.add(state);
                const nextStates = transitions[state] || [];
                for (const nextState of nextStates) {
                    const newPath = path.slice();
                    newPath.push(nextState);
                    queue.push(newPath);
                }
            }
        }
        return [];
    }

    resumeState(targetState, maxAttempts = 100) {
        console.log(`Set target state: ${targetState} (current is ${this.state})`);

        const nextAction = (task, targetState) => {
            const path = this.findPath(RosBaseState.transitions, task.state, targetState);
            if (path.length <= 1) return null;
            return path[1];
        };

        while (this.state !== targetState) {
            const cls = nextAction(this, targetState);
            if (cls === null) throw new Error('No next action! Unreachable!');
            if (maxAttempts < 0) throw new Error('Over max attempts!');

            console.log(`Current: ${this.state}, try to_${cls}`);
            var res = this[`to${cls}`]();
            maxAttempts -= 1;
            if(res?.constructor?.name=='Promise')break;
        }
        if(res?.constructor?.name=='Promise')res.then(controller=>{
            this.resumeState(targetState,maxAttempts);
        }).catch(
            e =>{
                console.log(e);
                this.resumeState(targetState,maxAttempts);
            }
        )

        if(this.state == targetState)console.log(`Success to target state: ${this.state}`);
    }
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
        const state = new RosBaseState(this);
        this.getROS = () => null;
        this.topic = () => null;
        this.state = () => state;
    }
    currentState(){return this.state().state}
    resumeState(targetState=RosBaseState.States.ConnectDone, maxAttempts = 100){
        this.state().resumeState(targetState,maxAttempts);
    }
    toFailure(){this.state().toFailure()}
    toNULL(){this.state().toNULL()}
    toClosed(){this.state().toClosed()}
    toConnecting(){this.state().toConnecting()}
    toConnectDone(){this.state().toConnectDone()}
    toClosing(){this.state().toClosing()}

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
            this.getROS = () => ros;

            this.getROS().on("error", error => {

                this.getROS = () => null;
                if (call_from_outter) this.connected_cnt++;
                callback.onError(error);
            });

            this.getROS().on("connection", () => {
                this.self_closed = false;
                if (call_from_outter) this.connected_cnt++;
                callback.onConnection();
            });

            this.getROS().on("close", () => {

                this.getROS = () => null;
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
            this.topic = () => topic;
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

        if (this.topic() && this.topic().unadvertise) this.topic().unadvertise();
        this.topic = () => null;

        if (this.getROS() && this.getROS().socket && this.getROS().socket.readyState !== WebSocket.CLOSED) {
            this.getROS().close();
        }

        this.getROS = () => null;
    }
}
