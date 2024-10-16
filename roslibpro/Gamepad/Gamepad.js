
export class EventDispatcher{

	constructor(){
		this._listeners = {};
	}

	addEventListener(type, listener){

		const listeners = this._listeners;

		if(listeners[type] === undefined){
			listeners[type] = [];
		}

		if(listeners[type].indexOf(listener) === - 1){
			listeners[type].push( listener );
		}

	}

	hasEventListener(type, listener){

		const listeners = this._listeners;

		return listeners[type] !== undefined && listeners[type].indexOf(listener) !== - 1;
	}

	removeEventListener(type, listener){

		let listeners = this._listeners;
		let listenerArray = listeners[type];

		if (listenerArray !== undefined){

			let index = listenerArray.indexOf(listener);

			if(index !== - 1){
				listenerArray.splice(index, 1);
			}
		}

	}

	removeEventListeners(type){
		if(this._listeners[type] !== undefined){
			delete this._listeners[type];
		}
	};

	dispatchEvent(event){

		let listeners = this._listeners;
		let listenerArray = listeners[event.type];

		if ( listenerArray !== undefined ) {
			//event.target = this;

			for(let listener of listenerArray.slice(0)){
        
				listener.call(this, event);
			}
		}

	}

}

class Gamepad extends EventDispatcher{
  constructor(data) {
    super();
    
    this.leftTrigger = data?.leftTrigger??0; 
    this.rightTrigger = data?.rightTrigger??0; 
    this.leftBumper = data?.leftBumper??false;
    this.rightBumper = data?.rightBumper??false;
   
    // Buttons
    this.buttonA = data?.buttonA??false;
    this.buttonB = data?.buttonB??false;
    this.buttonX = data?.buttonX??false;
    this.buttonY = data?.buttonY??false;
   
    // D-Pad
    this.dPadUp = data?.dPadUp??false;
    this.dPadDown = data?.dPadDown??false;
    this.dPadLeft = data?.dPadLeft??false;
    this.dPadRight = data?.dPadRight??false;
   
    // Sticks (axes)
    this.leftStickX = data?.leftStickX??0; // Range from -1 to 1
    this.leftStickY = data?.leftStickY??0; // Range from -1 to 1
   
    this.rightStickX = data?.rightStickX??0; // Range from -1 to 1
    this.rightStickY = data?.rightStickY??0; // Range from -1 to 1

    // Other Buttons
    this.start = data?.start??false;
    this.select = data?.select??false; // back
    this.home = data?.home??false;

    this.leftStickButton = false; // Clickable stick
    this.rightStickButton = false; // Clickable stick
  }

  diff_from(previous) {
    var is_diff = false;
    var data = new Gamepad();
    var keys = Object.keys(this).filter(k=>this[k] != previous[k] && k!='timestamp');
    if(keys.length>0)is_diff=true;
    keys.forEach(k=>data[k]=this[k]);    
    data.timestamp = this.timestamp;
    return {data,is_diff};
  }  
  
  // Method to update gamepad state from an input object
  update(buttonArray, axes) {
    const previous = new Gamepad({...this});

    // Triggers and Bumpers
    this.leftTrigger = buttonArray[6].value; // Range from 0 to 1
    this.rightTrigger = buttonArray[7].value; // Range from 0 to 1
    this.leftBumper = buttonArray[4].pressed;
    this.rightBumper = buttonArray[5].pressed;
   
    // Buttons
    this.buttonA = buttonArray[0].pressed;
    this.buttonB = buttonArray[1].pressed;
    this.buttonX = buttonArray[2].pressed;
    this.buttonY = buttonArray[3].pressed;
   
    // D-Pad
    this.dPadUp = buttonArray[12].pressed;
    this.dPadDown = buttonArray[13].pressed;
    this.dPadLeft = buttonArray[14].pressed;
    this.dPadRight = buttonArray[15].pressed;
   
    // Sticks (axes)
    this.leftStickX = axes[0]; // Range from -1 to 1
    this.leftStickY = axes[1]; // Range from -1 to 1 with direction UP-TO-DOWN
    this.rightStickX = axes[2]; // Range from -1 to 1
    this.rightStickY = axes[3]; // Range from -1 to 1 with direction UP-TO-DOWN
    
    // Other Buttons
    this.start = buttonArray[9].pressed;
    this.select = buttonArray[8].pressed; // back
    this.home = buttonArray[16].pressed;
    
    this.leftStickButton = buttonArray[10].pressed; // Clickable stick
    this.rightStickButton = buttonArray[11].pressed; // Clickable stick

    this.timestamp = Date.now();

    const current = this;

    this.dispatchEvent({'type':'update',current:current,previous:previous});
  }


  // Method to log the current state (for debugging)
  toString() {
    return [
      `Triggers: Left=${this.leftTrigger}, Right=${this.rightTrigger}`,
      `Bumpers: Left=${this.leftBumper}, Right=${this.rightBumper}`,
      `Buttons A, B, X, Y: A=${this.buttonA}, B=${this.buttonB}, X=${this.buttonX}, Y=${this.buttonY}`,
      `D-Pad: Up=${this.dPadUp}, Down=${this.dPadDown}, Left=${this.dPadLeft}, Right=${this.dPadRight}`,
      `Left Stick: X=${this.leftStickX}, Y=${this.leftStickY}`,
      `Right Stick: X=${this.rightStickX}, Y=${this.rightStickY}`,
      `Other Buttons: Start=${this.start}, Select=${this.select}, Home=${this.home}`,
      `Left Stick Button=${this.leftStickButton}, Right Stick Button=${this.rightStickButton}`
    ].join("\n");
  }

  toRosFormat() {
    const axes = new Array(6);
    axes[0] = this.leftStickX;
    axes[1] = this.leftStickY;
    axes[2] = -this.rightStickX;
    axes[3] = -this.rightStickY;
    axes[4] = this.dPadLeft ? -1.00 : (this.dPadRight ? 1.00 : 0.00);
    axes[5] = this.dPadUp ? 1.00 : (this.dPadDown ? -1.00 : 0.00);

    const data = new Array(6);
    data[0] = this.leftTrigger == 1 ? axes[1] : 0.00;
    data[1] = this.rightTrigger == 1 ? axes[3] : 0.00;
    data[2] = this.leftTrigger == 1 ? axes[0] : 0.00;
    data[3] = this.rightTrigger == 1 ? axes[2] : 0.00;
    data[4] = this.leftTrigger == 1 ? axes[5] : 0.00;
    data[5] = this.leftTrigger == 1 ? axes[4] : 0.00;
    return data;
  }
}

class GamepadDirectInput extends Gamepad{
  _povAzimuth = {
    N:  -1,
    NE: -1 + 2 / 7,
    E:  -1 + 4 / 7,
    SE: -1 + 6 / 7,
    S:  -1 + 8 / 7,
    SW: -1 + 10 / 7,
    W:  -1 + 12 / 7,
    NW:  1
  };

  update(buttonArray, axes) {
    
    const previous = new Gamepad({...this});

    const povAzimuth = this._povAzimuth;

    // Triggers and Bumpers
    this.leftTrigger = buttonArray[6].pressed ? 1 : 0; 
    this.rightTrigger = buttonArray[7].pressed ? 1 : 0; 
    this.leftBumper = buttonArray[4].pressed;
    this.rightBumper = buttonArray[5].pressed;
   
    // Buttons
    this.buttonA = buttonArray[2].pressed;
    this.buttonB = buttonArray[3].pressed;
    this.buttonX = buttonArray[0].pressed;
    this.buttonY = buttonArray[1].pressed;
   
    // D-Pad
    this.dPadUp = [povAzimuth.NW, povAzimuth.N, povAzimuth.NE].find((element) => axes[9].toFixed(3) == element.toFixed(3)) !== undefined;
    this.dPadRight = [povAzimuth.NE, povAzimuth.E, povAzimuth.SE].find((element) => axes[9].toFixed(3) == element.toFixed(3)) !== undefined;
    this.dPadDown = [povAzimuth.SE, povAzimuth.S, povAzimuth.SW].find((element) => axes[9].toFixed(3) == element.toFixed(3)) !== undefined;
    this.dPadLeft = [povAzimuth.SW, povAzimuth.W, povAzimuth.NW].find((element) => axes[9].toFixed(3) == element.toFixed(3)) !== undefined;

    // Sticks (axes)
    this.leftStickX = axes[0]; // Range from -1 to 1
    this.leftStickY = axes[1]; // Range from -1 to 1
   
    this.rightStickX = axes[2]; // Range from -1 to 1
    this.rightStickY = axes[5]; // Range from -1 to 1

    // Other Buttons
    this.start = buttonArray[11].pressed;
    this.select = buttonArray[10].pressed; // back
    this.home = buttonArray[12].pressed;

    this.leftStickButton = buttonArray[8].pressed; // Clickable stick
    this.rightStickButton = buttonArray[9].pressed; // Clickable stick

    this.timestamp = Date.now();

    const current = this;
    this.dispatchEvent({'type':'update',current:current,previous:previous});
  }
}


export class GamepadDriver extends EventDispatcher {
  constructor() {
      super();                  
      window.addEventListener("gamepadconnected",(event)=>{this.on_gamepadconnected(event)});
      window.addEventListener("gamepaddisconnected",(event)=>{this.on_gamepaddisconnected(event)});
  }

  get_gamepad_data() {
      const gamepadInventory = navigator.getGamepads ? navigator.getGamepads() : 
      (navigator.webkitGetGamepads ? navigator.webkitGetGamepads : []);
      const gamepad = gamepadInventory.filter(gamepad => gamepad)[0];
      if(!gamepad) {
          return null;
      }
      return gamepad;
  }

  stop_loop(){
      this.isAborted = true;
  }    
  loop(model=null) {
      this.isAborted = false;
      // Set up requestAnimationFrame with a fallback for older browsers
      this.rAF = window.mozRequestAnimationFrame || window.webkitRequestAnimationFrame ? 
                  (callback) => window.setTimeout(callback, 1000 / 60):null;

      // Define the main loop function
      const mainLoop = () => {
          if (this.isAborted) {
              return; // Exit loop if isAborted is true
          }

          // Retrieve gamepad data and dispatch custom event
          const gamepad = this.get_gamepad_data();
          model?.update(gamepad.buttons,gamepad.axes);
          // console.log(gamepad.axes);

          // Continue the loop
          this.rAF(mainLoop);
      };

      // Start the loop
      this.rAF(mainLoop);
  }
  on_gamepadconnected(event){
          this.dispatchEvent({type: "on_gamepadconnected",event: event});
  }
  on_gamepaddisconnected(event){
      this.dispatchEvent({type:'on_gamepaddisconnected',event:event});
  }                
  dispose(){
      window.removeEventListener("gamepadconnected",this.on_gamepadconnected);
      window.removeEventListener("gamepaddisconnected",this.on_gamepaddisconnected);
  }
}

export class GamepadFactory {
  static gamepad(gamepadType) {
    if(gamepadType === "DirectInput") {
      return new GamepadDirectInput();
    } else {
      return new Gamepad();
    }
  }
}

/// ---- controller

export class GamepadController extends EventDispatcher {
  constructor(model) {
    super();
    this.model = model;
    this.model.addEventListener('update',(event)=>{this._on_update(event)});
  } 
  
  _on_update(event){
    const current = event.current;
    const previous = event.previous;
    
    const {data,is_diff} = current.diff_from(new Gamepad());
    if(is_diff) {
      this.dispatchEvent({'type':'onValueChanged',gamepad:data});
    }
    if(data.start){      
      this.dispatchEvent({'type':'onPowered',gamepad:data});
    }
  }
  onValueChanged(callback) {
    this.addEventListener("onValueChanged", callback);
  }

  onPowered(callback) {
    this.addEventListener("onPowered", callback);
  }

  dispose() {
    this.removeEventListeners("onValueChanged");
    this.removeEventListeners("onPowered");
  }
};






