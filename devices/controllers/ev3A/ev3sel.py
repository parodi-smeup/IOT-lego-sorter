#!/usr/bin/env python3
# so that script can be run from Brickman

### THIS IS THE ONLY LINE TO MODIFY BETWEEN ONE EV3 AND ANOTHER, IT'S NAME IS NECESSARY FOR THE system.py ### 
### SCRIPT TO SEND IT THE RIGHT CONFIG MESSAGE ###
ev3_name = 'ev3A'
condition = 0
configuring = 0
brokerIP = "172.16.3.81"

# importing libraries
import paho.mqtt.client as mqtt
import ev3dev.ev3 as ev3dev               # control that < #!/usr/bin/env python > is the first line of the code
import random
import json
from time import sleep
from time import time
from datetime import datetime

# tuples

#colors = ('unknown', 'black', 'blue', 'green', 'yellow', 'red', 'white', 'brown')
colors = ('unknown', 'unknown', 'blue', 'unknown', 'yellow', 'red', 'white', 'yellow') #!!! baro, metto uguali unkn/black/green e yellow/brown!!!#
ports = ('in1', 'in2', 'in3', 'in4', 'outA', 'outB', 'outC', 'outD')
time0 = time()
ev3_peripherals = []
ev3 = None
mA = None
mB = None
mC = None
mD = None
s1 = None
s2 = None
s3 = None
s4 = None

class Motor:

    types = ('Medium', 'Large')
    motors_list = []

    def __init__(self, type, port, name):
        if type == 'Medium':
            Motor.motors_list.append(self)
            self.name = name
            self.ev3motor = ev3dev.MediumMotor(port)
        elif type == 'Large':
            Motor.motors_list.append(self)
            self.name = name
            self.ev3motor = ev3dev.LargeMotor(port)
        else:
            self = None


class Sensor:

    sensors_list = []

    def __init__(self, port, name):
        Sensor.sensors_list.append(self)
        self.ev3sensor = ev3dev.ColorSensor(port)
        self.ev3sensor.mode = 'COL-COLOR'
        self.name = name
        self.color_seen = colors[0]

class TouchSensor:

    touchsensors_list = []

    def __init__(self, port, name):
        TouchSensor.touchsensors_list.append(self)
        self.ev3sensor = ev3dev.TouchSensor(port)
        self.ev3sensor.mode = 'TOUCH'
        self.name = name
        self.is_pressed = False

class Ev3:             # defining "Ev3" class

    global mA, mB, mC, mD, s1, s2, s3, s4

    #       EX:       ev3_1,'motor_station', 'Large'   
    def __init__(self, name, motorA=None, typeA=None, motorB=None, typeB=None, motorC=None, typeC=None, motorD=None, typeD=None,
    sensor1=None, sensor2=None, sensor3=None, sensor4=None):   #function called whenever an instance of the class station is initialized
        Sensor.sensors_list = []
        Motor.motors_list = []        
        self.name = name                                             # object attribute: name of the ev3

        if (not typeA in Motor.types) and (typeA != None):           # if no motor is plugged in this port None is passed as default value
            print('Assigned motor type is not valid.')               # as input of ev3 init

        if (not isinstance(motorA, str)) and (motorA != None):        
            print('Assigned name is not a string.')
        elif motorA == None:
            pass
        else:                                                        # if a motor is plugged in port A an instance of the class Motor 
            mA = Motor(typeA, 'outA', motorA)                        # is created


        if (not typeB in Motor.types) and (typeB != None):
            print('Assigned motor type is not valid.')

        if (not isinstance(motorB, str)) and (motorB != None):        
            print('Assigned name is not a string.')
        elif motorB == None:
            pass            
        else:
            mB = Motor(typeB, 'outB', motorB)


        if (not typeC in Motor.types) and (typeC != None):
            print('Assigned motor type is not valid.')

        if (not isinstance(motorC, str)) and (motorC != None):        
            print('Assigned name is not a string.')
        elif motorC == None:
            pass
        else:
            mC = Motor(typeC, 'outC', motorC)


        if (not typeD in Motor.types) and (typeD != None):
            print('Assigned motor type is not valid.')

        if (not isinstance(motorD, str)) and (motorD != None):        
            print('Assigned name is not a string.')
        elif motorD == None:
            pass
        else:
            mD = Motor(typeD, 'outD', motorD) 


        if (not isinstance(sensor1, str)) and (sensor1 != None):        
            print('Assigned name is not a string.')
        elif sensor1 == None:
            pass
        else:
            s1 = Sensor('in1', sensor1)

        if (not isinstance(sensor2, str)) and (sensor2 != None):        
            print('Assigned name is not a string.')
        elif sensor2 == None:
            pass
        else:
            s2 = Sensor('in2', sensor2)

        if (not isinstance(sensor3, str)) and (sensor3 != None):        
            print('Assigned name is not a string.')
        elif sensor3 == None:
            pass
        else:
            s3 = Sensor('in3', sensor3) 


        if (not isinstance(sensor4, str)) and (sensor4 != None):        
            print('Assigned name is not a string.')
        elif sensor4 == None:
            pass    
        elif sensor4 == 'stop':                 ### <--- stop sensor / red button can only be plugged in port 'in4' and be named 'stop'
            s4 = TouchSensor('in4', sensor4)
        else:    
            s4 = Sensor('in4', sensor4)
        
        print(Sensor.sensors_list[0].name)
        topic = 'ev3_configured'
        payload = {'name':name}
        payload = json.dumps(payload, indent=4)
        client.publish(topic, payload, 2)

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    if rc==0:
        print("Connected OK Returned code = ",rc)
        client.subscribe('stop', 2)
        client.subscribe('sensor/on_demand/question', 2)
        client.subscribe('motor/action/stop', 2)
        client.subscribe('motor/action/rel', 2)
        client.subscribe('motor/action/rel1verse', 2)
        client.subscribe('motor/action/forever', 2)
        client.subscribe('motor/action/timed', 2)
        client.subscribe('motor/action/abs', 2)
        client.subscribe('time0_init', 2)
        client.subscribe('ev3_config', 2)

    else:
        print("Bad connection Returned code = ",rc)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print('Message received:')
    global condition
    global configuring
    global ev3
    global ev3_peripherals
    global ev3_name

    if msg.topic == 'time0_init':
        payload = msg.payload.decode("utf-8","ignore")      # transform payload into str
        payload = json.loads(payload)                       # transforms payload into dict
        print (msg.topic)
        print (payload)
        print('Topic: '+msg.topic+'. Initializing the system time.')
        global time0
        time0 = time()

    elif msg.topic == 'ev3_config':
        payload = msg.payload.decode("utf-8","ignore")      # transform payload into str
        payload = json.loads(payload)                       # transforms payload into dict
        if payload['ev3'] == ev3_name:
            configuring = True            
            print('Topic: '+msg.topic+'. Configuring the Ev3.')
            print (payload)
            ev3 = Ev3(ev3_name, motorA = payload["1"], typeA=payload["2"], motorB = payload["3"],
            typeB=payload["4"], motorC = payload["5"], typeC=payload["6"], motorD = payload["7"],
            typeD=payload["8"], sensor1 = payload["9"], sensor2 = payload["10"], sensor3 = payload["11"],
            sensor4 = payload["12"])
            condition = True
            configuring = False

    elif msg.topic == 'stop':
        condition = False
        print('Topic: '+msg.topic+'. Stopping all motors and exiting the script.')
        for motor in Motor.motors_list:        
            motor.ev3motor.stop(stop_action = 'coast')

    elif msg.topic == 'sensor/on_demand/question':          # this topic identifies the request to a sensor to give its output measure
        payload = msg.payload.decode("utf-8","ignore")      # transform payload into str
        payload = json.loads(payload)                       # transforms payload into dict
        print (msg.topic)
        print (payload)        
        for sensor in Sensor.sensors_list:
            if payload['ev3'] == ev3.name:
                if payload['sensor'] == sensor.name:
                    print('Requested color seen by sensor '+sensor.name)
                    color = colors[sensor.ev3sensor.value()]    # measuring the color seen by the sensor
                    topic = 'sensor/on_demand/answer'
                    timenow = time()
                    ts = timenow - time0                        # tempo passato dall'apertura del file .py
                    payload_out = {'ev3': ev3.name, 'sensor': sensor.name, 'value': color, 'ts': ts}
                    payload_out = json.dumps(payload_out, indent=4)
                    client.publish(topic, payload_out, 2)

    elif msg.topic == 'motor/action/rel':                   # this topic identifies the action (rel_pos -> +240, rel_pos ->-240)
        payload = msg.payload.decode("utf-8","ignore")      # transform payload into str
        payload = json.loads(payload)                       # transforms payload into dict
        print (msg.topic)
        print (payload)        
        for motor in Motor.motors_list:        
            if payload['ev3'] == ev3.name:                  # checks if this ev3 is the addressee of the published message
                if payload['motor'] == motor.name:          # looks for the right motor to activate
                    print('Requested motor '+motor.name+' activation.')
                    angle = payload['value']
                    motor.ev3motor.run_to_rel_pos(position_sp = angle, speed_sp = 500, stop_action = 'hold')
                    sleep(1)
                    motor.ev3motor.run_to_rel_pos(position_sp = -angle, stop_action = 'hold')

    elif msg.topic == 'motor/action/rel1verse':                   # this topic identifies the action (rel_pos -> +240, rel_pos ->-240)
        payload = msg.payload.decode("utf-8","ignore")      # transform payload into str
        payload = json.loads(payload)                       # transforms payload into dict
        print (msg.topic)
        print (payload)        
        for motor in Motor.motors_list:        
            if payload['ev3'] == ev3.name:                  # checks if this ev3 is the addressee of the published message
                if payload['motor'] == motor.name:          # looks for the right motor to activate
                    print('Requested motor '+motor.name+' activation.')
                    value = payload['value']
                    speed = value[0]
                    angle = value[1]
                    motor.ev3motor.run_to_rel_pos(position_sp = angle, speed_sp = speed, stop_action = 'hold')

    elif msg.topic == 'motor/action/abs':                   # this topic identifies the action (abs_pos -> +240, abs_pos ->-240)
        payload = msg.payload.decode("utf-8","ignore")      # transform payload into str
        payload = json.loads(payload)                       # transforms payload into dict
        print (msg.topic)
        print (payload)        
        for motor in Motor.motors_list:        
            if payload['ev3'] == ev3.name:                  # checks if this ev3 is the addressee of the published message
                if payload['motor'] == motor.name:          # looks for the right motor to activate
                    print('Requested motor '+motor.name+' activation.')
                    value = payload['value']
                    angle1 = value[0]
                    angle2 = value[1]
                    motor.ev3motor.run_to_abs_pos(position_sp = angle1, speed_sp = 500, stop_action = 'hold')
                    sleep(2)
                    motor.ev3motor.run_to_abs_pos(position_sp = angle2, stop_action = 'hold')

    elif msg.topic == 'motor/action/timed':                   # this topic identifies the action (rel_pos -> +240, rel_pos ->-240)
        payload = msg.payload.decode("utf-8","ignore")      # transform payload into str
        payload = json.loads(payload)                       # transforms payload into dict
        print (msg.topic)
        print (payload)        
        for motor in Motor.motors_list:        
            if payload['ev3'] == ev3.name:                  # checks if this ev3 is the addressee of the published message
                if payload['motor'] == motor.name:          # looks for the right motor to activate
                    print('Requested motor '+motor.name+' activation.')
                    value = payload['value']
                    t = value[0]
                    speed = value[1]
                    motor.ev3motor.run_timed(speed_sp = speed, time_sp = t, stop_action = 'brake')

    elif msg.topic == 'motor/action/forever':               # this topic identifies the action (run forever (speed specified in 'value'))
        payload = msg.payload.decode("utf-8","ignore")      # transform payload into str
        payload = json.loads(payload)                       # transforms payload into dict
        print (msg.topic)
        print (payload)        
        for motor in Motor.motors_list:        
            if payload['ev3'] == ev3.name:                  # checks if this ev3 is the addressee of the published message
                if payload['motor'] == motor.name:          # looks for the right motor to activate
                    print('Requested motor '+motor.name+' activation.')
                    speed = int(payload['value'])
                    motor.ev3motor.run_forever(speed_sp = speed)

    elif msg.topic == 'motor/action/stop':                  # this topic identifies the action (stop (stop_action specified in 'value'))
        payload = msg.payload.decode("utf-8","ignore")      # transform payload into str
        payload = json.loads(payload)                       # transforms payload into dict
        print (msg.topic)
        print (payload)        
        for motor in Motor.motors_list:        
            if payload['ev3'] == ev3.name:                  # checks if this ev3 is the addressee of the published message
                if payload['motor'] == motor.name:          # looks for the right motor to activate
                    print('Requested motor '+motor.name+' activation.')
                    how = payload['value']
                    motor.ev3motor.stop(stop_action = how)

    else:
        print('Message topic not recognised.')
                    
# The callback for SUBSCRIBE.
def on_subscribe(client, userdata, mid, granted_qos):
    print('Successfully subscribed!')

# The callback for when a PUBLISH message is sent to the broker.
def on_publish(client, userdata, mid):
    print("Message has been published.")

# initialize mqtt connection
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.on_subscribe = on_subscribe
client.on_publish = on_publish

client.connect(brokerIP)          # Broker on PC (IP fixed: 192.168.0.2)
 
sleep (0.1)                            # Waiting for the connection to be set     

client.loop_start()

topic = 'hello'
payload = {'name':ev3_name}
payload = json.dumps(payload, indent=4)
client.publish(topic, payload, 2)

while not condition:
    sleep(3)

while condition:
    if configuring == True:
        sleep(5)
    for sensor in Sensor.sensors_list:
        color = colors[sensor.ev3sensor.value()]    
        if color != sensor.color_seen:
            topic = 'sensor/on_change'                              # this topic identifies the sensors continuous stream of pubblications 
            print('Color seen by sensor '+sensor.name+' has changed. Publishing its value.')    # of their output when it changes
            timenow = time()
            ts = timenow - time0                                    # tempo passato dall'apertura del file .py
            payload_out = {'ev3': ev3.name, 'sensor': sensor.name, 'value': color, 'ts': ts}
            payload_out = json.dumps(payload_out, indent=4)
            client.publish(topic, payload_out, 2)
            sensor.color_seen = color
            if sensor.name == 'rfid_sensor':
                topic = 'sensor/rfid'                              # this topic identifies the sensors continuous stream of pubblications 
                print('Color seen by sensor '+sensor.name+' has changed. Publishing its value.')    # of their output when it changes
                timenow = time()
                ts = timenow - time0                                    # tempo passato dall'apertura del file .py
                payload_out = {'ev3': ev3.name, 'sensor': sensor.name, 'value': color, 'ts': ts}
                payload_out = json.dumps(payload_out, indent=4)
                client.publish(topic, payload_out, 2)   
    for sensor in TouchSensor.touchsensors_list:
        sensor.is_pressed = sensor.ev3sensor.is_pressed
        if sensor.is_pressed == True:
            print('eccoci')
            topic = 'stop'
            timenow = time()
            ts = timenow - time0
            payload_out = True
            client.publish(topic, payload_out, 2)
    sleep(0.1)                                                      # sensors output refreshing happens every ~0.1 sec 

print('Ending listening. Closing script.')
client.loop_stop()
