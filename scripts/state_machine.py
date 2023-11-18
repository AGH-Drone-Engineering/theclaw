import rospy
from std_msgs.msg import Int32

# Tu powinny być zdefiniowane funkcje detectTeddy(), goToBearArea(), getDistance(), closeEnough(), sendCmdVel(), setGripperAngle(alpha), goHome()

class IdStan:
    SZUKAJ_MIS = 1
    IDZ_DO_MIS = 2 
    CHWYT = 3
    UCIEKAJ = 4

class StateMachine:
    def __init__(self):
       
        self.CURRENT_STATE = IdStan.SZUKAJ_MIS
        self.PREVIOUS_STATE = IdStan.SZUKAJ_MIS

        rospy.init_node('teddy_subscriber', anonymous=True)  
        self.stan_sub = rospy.Subscriber("teddy_topic", Int32, self.callback_state)  
        self.teddy_pub = rospy.Publisher('teddy_topic', Int32, queue_size=10)
        teddy_msg = Int32()
        teddy_msg.data = self.CURRENT_STATE
        self.teddy_pub.publish(teddy_msg)

    def main_loop(self):


        while not rospy.is_shutdown():
            #chodzeie do misia
            if self.CURRENT_STATE == IdStan.SZUKAJ_MIS:
                self.seek_bear()
                teddy_msg = Int32()
                teddy_msg.data = IdStan.IDZ_DO_MIS
            
            #chwytanie misia
            elif self.CURRENT_STATE == IdStan.IDZ_DO_MIS:
                self.go_to_bear()
                teddy_msg = Int32()
                teddy_msg.data = IdStan.CHWYT
            
            #chwyt misia

            elif self.CURRENT_STATE == IdStan.CHWYT:
                self.catch_bear()
                teddy_msg = Int32()
                teddy_msg.data = IdStan.UCIEKAJ
            
            #ucieczka 
            
            elif self.CURRENT_STATE == IdStan.UCIEKAJ:
                self.run_away()
                teddy_msg = Int32()
                teddy_msg.data = IdStan.SZUKAJ_MIS


            self.teddy_pub.publish(teddy_msg)
            # save previous state
            self.PREVIOUS_STATE = self.CURRENT_STATE

    def seek_bear(self):
        goToBearArea()
        # self.CURRENT_STATE = IdStan.IDZ_DO_MIS


    def go_to_bear(self):
        vforward = 0.1 # do dostosowania
        vangle = 10   # w stopniach do dostosowania
        while getDistance() > 0.1: # do dostosowania
            # dist= getDistance() # for scaling later
            if TURN_LEFT:
                sendCmdVel(vforward, vangle)
            elif TURN_RIGHT:
                sendCmdVel(vforward, -vangle)
            else:
                sendCmdVel(vforward, 0)

        
        self.CURRENT_STATE = IdStan.CHWYT


    def catch_bear(self):
        print("Wykryto misia blisko, łapiemy go...")
        setGripperAngle(alpha)
        self.CURRENT_STATE = IdStan.UCIEKAJ

    def run_away(self):
        print("Uciekaj!")
        goHome()
        return None
    
    def callback(self, data):
        if self.PREVIOUS_STATE != self.CURRENT_STATE:
            rospy.loginfo("Otrzymano: %d", data.data)
        # rospy.loginfo("Otrzymano: %d", data.data)  # Wyświetlenie odebranej wiadomości
        # Tutaj umieść logikę obsługi odebranej wiadomości (np. zmiana flagi bear_found)
        if data.data == 1:  
            teddy_msg = Int32()
            teddy_msg.data = self.CURRENT_STATE
            self.teddy_pub.publish(teddy_msg)

if __name__ == "__main__":
    state_machine = StateMachine()
    state_machine.main_loop()
    
