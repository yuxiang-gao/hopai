
import rospy
import tf
from hop_msgs.msg import EnemyPos
from hop_msgs.msg import GimbalAngle
from hop_msgs.msg import Odometry
from hop_msgs.msg import Path



class StackFSM:
    private var stack :Array;
 
    public function StackFSM() {
        this.stack = new Array();
    }
 
    public function update() :void {
        var currentStateFunction :Function = getCurrentState();
 
        if (currentStateFunction != null) {
            currentStateFunction();
        }
    }
 
    public function popState() :Function {
        return stack.pop();
    }
 
    def pushState(state :Function)
        if (getCurrentState() != state) 
            stack.push(state);
        
    
 
    def getCurrentState() 
        return stack.length > 0 ? stack[stack.length - 1] : null;
    

