### IMPORTING LIBRARIES ###
import paho.mqtt.client as mqtt
import random
import json
from time import sleep
from time import time
from datetime import datetime

### GLOBAL CONSTANTS ### 
brokerIP = "172.16.3.81"

colors = ('unknown','black','blue','green','yellow','red','white','brown')
gcolors = ('blue','yellow','red','white','brown')
ports = ('in1', 'in2','in3','in4', 'outA','outB','outC','outD')
peripherals = [None,None,None,None,None,None,None,None,None,None,None,None]
configured = [1, 2]

ConveyorSpeed = 500
ConveyorAngle = 1050

ColorTimerLimit = 7
RfidTimerLimit = 7
ColorToRfidTimerLimit = 6

ColorAngle0 = 0
ColorAngleV = 55
ColorAngleX = -100

RfidAngle = 450
RfidAngle0 = 0
RfidConveyorSpeed = +200
RfidConveyorAngle = +1400

### GLOBAL VARIABLES ###
time0 = time()

processed_order = 0
order = {}
orders = []                         # list containing the orders to be processed 
orders_received = []                # list containing the received jsons dicts that contain the orders (for orders traceability)

condition = False                   # while loop condition, modified into True by "stop" message reception
answer_received = True              # while loop condition for allowing on_demand answer to be registered
rfid_1_empty = True                 # while loop condition for allowing dispatch of the ball to order 1, modified by "on_change" message receprion from sensor "rfid_1"
rfid_2_empty = True                 # while loop condition for allowing dispatch of the ball to order 2, modified by "on_change" message receprion from sensor "rfid_2"
rfid_scraps_empty = True            # while loop condition for allowing dispatch of the ball to scraps, modified by "on_change" message receprion from sensor "rfid_scraps"

color_timer = 0.0                   # timer for timeout condition in "color-check" station (ball not entered or not recognised)
color_timer_start = 0.0             # time of start for color_timer
rfid_timer = 0.0                    # timer for timeout condition in "rfid-check" station (ball not entered or not recognised)
rfid_timer_start = 0.0              # time of start for rfid_timer
color_to_rfid_timer = 0.0
color_to_rfid_timer_start = 0.0

color_data = ''                     # str containing the color seen by color sensor
rfid_data = ''                      # str containing the rfid tag read by rfid sensor

j = 0                               # condition for correct dispatching of the ball to the right output channel (order1, order2, scraps)

### STATION STATES ###
color_sensor_available = True       # state of the "color-check" station: True = station idle, False = station working
rfid_present = False                # state of the "rfid-check" station: False = station idle, True = station working

### ORDERS STATE ###
order1_finished = False             # variable that triggers the "order1_finished" message
order2_finished = False             # variable that triggers the "order2_finished" message

### COUNTERS INIT ###
counter_in = 0                      # counts the balls entering the system, published on-change
counter_out = 0                     # counts the balls leaving the system in any way, published on-change
color_scrap_count = 0               # counts the balls sent to scrap at "color-check" station, published on-change
rfid_scrap_count = 0                # counts the balls sent to scrap at "rfid-check" station, published on-change
order1_count = 0                    # counts the balls sent to order 1 at "rfid-check" station, published on-change
order2_count = 0                    # counts the balls sent to order 2 at "rfid-check" station, published on-change
                                    # Note: color_scrap_count + rfid_scrap_count + order1_count + order2_count should be = counter_out
counter_color = 0                   # counts the balls processed by the "color-check" station (should be = counter_in)
counter_rfid = 0                    # counts the balls processed by the "rfid_check" station (should be = counter_in - color_scrap_count)
batch_pieces = 0                    # counts the balls charged in the upstream buffer

def pub_config():
    topic='ev3_config'
    timenow = time()
    ts = timenow - time0
    peripherals = ['conveyor', 'Large', 'color_select', 'Medium', 'rfid_conveyor', 'Large', 'rfid_select', 'Medium',
    'input_sensor', 'color_sensor', 'rfid_sensor', None]
    payload_out = {'ev3': 'ev3A', 'ts':ts}
    i = 1
    for element in peripherals:
        payload_out[i] = peripherals[i-1]
        i += 1        
    payload_out = json.dumps(payload_out, indent=4)
    client.publish(topic, payload_out, 2)

    timenow = time()
    ts = timenow - time0
    peripherals = [None, None, None, None, None, None, None, None,
    'rfid_1', 'rfid_2', 'rfid_scraps', 'stop']
    payload_out = {'ev3': 'ev3B', 'ts':ts}
    i = 1
    for element in peripherals:
        payload_out[i] = peripherals[i-1]
        i += 1        
    payload_out = json.dumps(payload_out, indent=4)
    client.publish(topic, payload_out, 2)

### CLASSES DEFINITION ###

class Motor:

    ## Class attributes ##
    motors_list = []                # attribute (type list) containing all the instances of the class Motor

    def __init__(self, name, ev3):

        ## instance attributes ##
        self.name = name
        self.ev3 = ev3

        # update motors_list at every new Motor instance creation #
        Motor.motors_list.append(self)

    # Instance method for publishing the command message to activate the motor in mode rel (default angle +240 / -240 deg) #
    def rel(self, angle=240):
        topic = 'motor/action/rel'
        timenow = time()
        ts = timenow - time0
        payload_out = {'ev3': self.ev3, 'motor': self.name,'value': angle, 'ts':ts}
        payload_out = json.dumps(payload_out, indent=4)
        client.publish(topic, payload_out, 2)

# Instance method for publishing the command message to activate the motor in mode rel (default angle +240 / -240 deg) #
    def rel1verse(self, speed=400, angle=240):
        topic = 'motor/action/rel1verse'
        timenow = time()
        ts = timenow - time0
        payload_out = {'ev3': self.ev3, 'motor': self.name,'value': [speed, angle], 'ts':ts}
        payload_out = json.dumps(payload_out, indent=4)
        client.publish(topic, payload_out, 2)

    # Instance method for publishing the command message to activate the motor in mode rel (default angle +240 / -240 deg) #
    def abs(self, angle1=240, angle2=0):
        topic = 'motor/action/abs'
        timenow = time()
        ts = timenow - time0
        payload_out = {'ev3': self.ev3, 'motor': self.name,'value': [angle1, angle2], 'ts':ts}
        payload_out = json.dumps(payload_out, indent=4)
        client.publish(topic, payload_out, 2)

    # Instance method for publishing the command message to activate the motor in mode run forever (default speed = 850 deg/s) #    
    def forever(self, speed=850):                   
        topic = 'motor/action/forever'
        timenow = time()
        ts = timenow - time0
        payload_out = {'ev3': self.ev3, 'motor': self.name,'value': speed, 'ts':ts}
        payload_out = json.dumps(payload_out, indent=4)
        client.publish(topic, payload_out, 2)

    # Instance method for publishing the command message to activate the motor in mode run timed (default time = 5s, default speed = -1000 deg/s) # 
    def timed(self, t=5000, speed = -1000):
        topic = 'motor/action/timed'
        timenow = time()
        ts = timenow - time0
        payload_out = {'ev3': self.ev3, 'motor': self.name,'value': [t, speed], 'ts':ts}
        payload_out = json.dumps(payload_out, indent=4)
        client.publish(topic, payload_out, 2)        

    # Instance method for publishing the command message to stop the motor (default mode = brake) #
    def stop(self, stop_action='brake'):
        topic = 'motor/action/stop'
        timenow = time()
        ts = timenow - time0
        payload_out = {'ev3': self.ev3, 'motor': self.name,'value': stop_action, 'ts':ts}
        payload_out = json.dumps(payload_out, indent=4)
        client.publish(topic, payload_out, 2)

class Sensor:

    ## Class attributes ##
    sensors_list = []               # class attribute (type list) containing all the instances of the class Sensor

    def __init__(self, name):

        ## instance attributes ##
        self.name = name
        # instance attribute (type dict) containing the last value measured by the sensor (updated on-change) #
        self.output = {'sensor':name, 'value':'None', 'ts':time0}

        # update sensors_list at every new Sensor instance creation #
        Sensor.sensors_list.append(self)

    # Instance method for publishing the command message to ask the physical sensor a new reading of the measured value #
    def read(self):
        topic = 'sensor/on_demand/question'
        timenow = time()
        ts = timenow - time0
        payload_out = {'sensor': self.name, 'ts':ts}
        payload_out = json.dumps(payload_out, indent=4)
        client.publish(topic, payload_out, 2)
        # waits for the message to be question to be answered
        global answer_received
        while answer_received:
            sleep(0.1)
        answer_received = True

# Subclass of Class Sensor #
class ColorSensor (Sensor):

    def __init__(self, name, ev3):

        # calls Superclass Sensor __init__ method for execution
        super().__init__(name)

        ## instance attributes ##
        # add .ev3 instance attribute containing the ev3 which in the sensor is plugged
        self.ev3 = ev3
        # update .output instance attribute (dict) with the key 'ev3'
        self.output['ev3'] = ev3

    # Instance method for publishing the command message to ask the physical sensor a new reading of the measured value (update of the superclass method)#
    def read(self):
        topic = 'sensor/on_demand/question'
        timenow = time()
        ts = timenow - time0
        # adds to payload_out the key 'ev3' typical of ColorSensor class only
        payload_out = {'ev3': self.ev3, 'sensor': self.name, 'ts':ts}
        payload_out = json.dumps(payload_out, indent=4)
        client.publish(topic, payload_out, 2)
        # waits for the message to be question to be answered
        global answer_received
        while answer_received:
            sleep(0.1)
        answer_received = True

# Subclass of Class Sensor #
class RfidSensor (Sensor):

    def __init__(self, name):

        # calls Superclass Sensor __init__ method for execution
        super().__init__(name)

    # NOT DEFINED UP TO NOW THE RFID SENSOR #

class Ev3:

    ## Class attributes ##
    ev3s_list = []               # class attribute (type list) containing all the instances of the class Ev3

    def __init__(self, name):

        ## instance attributes ##
        self.name = name

        # update ev3s_list at every new Ev3 instance creation #
        Ev3.ev3s_list.append(self)

### MQTT CALLBACKS (FUNCTIONS) ###

# The callback for when the client receives a CONNACK response from the server
def on_connect(client, userdata, flags, rc):
    if rc==0:
        print("Connection OK. Returned code = ",rc)
        client.subscribe('stop', 2)
        client.subscribe('sensor/on_demand/answer', 2)
        client.subscribe('sensor/on_change', 2)
        client.subscribe('sensor/rfid', 2)
        client.subscribe('hello', 2)
        client.subscribe('time0_init', 2)
        client.subscribe('ev3_configured', 2)

    else:
        print("Bad connection. Returned code = ",rc)


# The callback for when a PUBLISH message is received from the server
def on_message(client, userdata, msg):

    print('Message received:')
    global time0
    global condition


    # 'hello' topic identifies the connection to the system of a physical ev3 (with relative motors and sensors)
    # the system script thus have to instantiate in the rigth Classes all this new hardware connected to the system
    if msg.topic == 'hello':
        
        #payload = {
        #           'name' : ev3name,
        #           }

        payload = msg.payload.decode("utf-8","ignore")      # transforms payload into str
        payload = json.loads(payload)                       # transforms payload into dict

        print('A new ev3 is connected to the local network, it is:')
        print(payload)

        # publishes a message for time0 initialization to communicate the connected ev3 the value of system file time0
        topic = 'time0_init'
        payload_out = {'ev3': payload['name'], 'value': time()}
        payload_out = json.dumps(payload_out, indent=4)
        client.publish(topic, payload_out, 2)

        # an instance of class Ev3 is created
        ev3 = Ev3(payload['name'])
        pub_config()

    # 'stop' topic indentifies the system shutdown --> system.py script ceases execution
    elif msg.topic == 'stop':
        condition = False
        print('Topic: '+msg.topic+'. Exiting the script.')

    elif msg.topic == 'ev3_configured':
        print('A new ev3 has been configured, it is:')
        payload = msg.payload.decode("utf-8","ignore")      # transforms payload into str
        payload = json.loads(payload)                       # transforms payload into dict
        name = payload['name']
        print(name)
        configured.pop(0)
        if configured == []:
            condition = True

    # 'sensor/on_demand/answer' topic identifies the answwer from a sensor for an on-demand measure
    elif msg.topic == 'sensor/on_demand/answer':

        # payload = {
        #            'ev3': ev3.name, 
        #            'sensor': sensor.name, 
        #            'value': color seen, 
        #            'ts': time stamp
        #            }

        payload = msg.payload.decode("utf-8","ignore")      # transforms payload into str
        payload = json.loads(payload)                       # transforms payload into dict

        # updates the Sensor instance attribute .output (dict)
        for sensor in Sensor.sensors_list:
            if payload['sensor'] == sensor.name:
                sensor.output = payload

        print('Received answer: ')
        print(payload)
        # condition answer_received = True blocks the sensor method .read waiting for the sensor.output to be updated
        global answer_received
        answer_received = False


    # 'sensor/on_change' topic identifies the messages automatically published by sensors for updating their output on-change
    elif msg.topic == 'sensor/on_change':

        # payload = {
        #            'ev3': ev3.name, 
        #            'sensor': sensor.name, 
        #            'value': color seen, 
        #            'ts': time stamp
        #            }
               
        payload = msg.payload.decode("utf-8","ignore")      # transforms payload into str
        payload = json.loads(payload)                       # transforms payload into dict

        # updates the Sensor instance attribute .output (dict)
        for sensor in Sensor.sensors_list:
            if payload['sensor'] == sensor.name:
                sensor.output = payload
                break

        # modifies the while loop conditions for right balls output channels selection (order1, order2, scraps)
        if payload['sensor'] == 'rfid_1' and payload['value'] in gcolors and processed_order == 1:
            global rfid_1_empty
            rfid_1_empty = False

        if payload['sensor'] == 'rfid_2' and payload['value'] in gcolors and processed_order == 2:
            global rfid_2_empty
            rfid_2_empty = False

        if payload['sensor'] == 'rfid_scraps' and payload['value'] in gcolors and processed_order == 0:
            global rfid_scraps_empty
            rfid_scraps_empty = False


    # 'sensor/rfid' topic identifies the message published on-change by RFID sensor
    elif msg.topic == 'sensor/rfid':

        # payload = { 
        #            'sensor': sensor.name, 
        #            'value': RFID tag read, 
        #            'ts': time stamp
        #            }

        payload = msg.payload.decode("utf-8","ignore")      # transforms payload into str
        payload = json.loads(payload)                       # transforms payload into dict

        # updates the Sensor instance attribute .output (dict)
        for sensor in Sensor.sensors_list:
            if payload['sensor'] == sensor.name:
                sensor.output = payload

        print('Received answer: ')
        print(payload)

        # modifies the variable rfid_present identifying the state of the "rfid-check" station
        if payload['value'] in gcolors:
            global rfid_present
            rfid_present = True
            topic = 'selezionatori/states'
            timenow = time()
            ts = timenow
            payload_out = {'type': 'int', 'title': 'rfid_station_state', 'value': int(rfid_present), 'ts':ts, 'selezionatore':'polimi'}
            payload_out = json.dumps(payload_out, indent=4)
            client.publish(topic, payload_out, 2)             


    # 'order' topic identifies a message communicating a new order to be processed by the system
    elif msg.topic == 'selezionatori/orders':

        # payload = { 
        #            'order': 1
        #            'nome': Ord001,
        #            'tipo': 
        #            'value': {
        #                        1: ['color', RFID Tag]
        #                        2: ['color', RFID Tag]
        #                        ...
        #                           } 
        #            'ts': time stamp
        #            }        

        payload = msg.payload.decode("utf-8","ignore")      # transforms payload into str
        payload = json.loads(payload)                       # transforms payload into dict

        # updates orders and orders_received lists
        orders_received.append(payload)            # list containing the received jsons dicts that contain the orders (for orders traceability)
        order = payload['content']
        orders.append(order)                       # list containing the orders to be processed

        # modifies global variables order_finished to update order status
        global order1_finished
        global order2_finished
        order1_finished = False         # <-- proprietà dell'ordine
        order2_finished = False        

        print('Received new order')
        print(payload)

    # 'time0_init' topic identifies the time0 initialization message published every time a new hardware device is connected to the net
    elif msg.topic == 'time0_init':

        # payload = {
        #            'ev3': just connected ev3 name,
        #            'value': time of message pubblication
        #            }

        payload = msg.payload.decode("utf-8","ignore")      # transform payload into str
        payload = json.loads(payload)                       # transforms payload into dict
        print('Topic: '+msg.topic+'. Initializing the system time.')
        print (payload)
        # since all the connected devices receive time0_init msg, they set time0 value MORE OR LESS at the same time
        time0 = time()

    else:
        print('Message topic not recognised.')


# The callback for SUBSCRIBE.
def on_subscribe(client, userdata, mid, granted_qos):
    print('Successfully subscribed!')


# The callback for when a PUBLISH message is sent to the broker.
def on_publish(client, userdata, mid):
    print("Message has been published.")

# initializing mqtt connection
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.on_subscribe = on_subscribe
client.on_publish = on_publish

client.connect(brokerIP)      # Broker on PC (IP fixed: 192.168.0.2)

client.loop_start()
print('Beginning Listening...')

### FUNCTIONS DEFINITIONS ###

def pub(topic, type, title, value):
    timenow = time()
    global time0
    ts = timenow - time0
    payload_out = {'type': type, 'title': title, 'value': value, 'ts':ts, 'selezionatore':'polimi'}
    payload_out = json.dumps(payload_out, indent=4)
    client.publish(topic, payload_out, 2)

### DEFINITION OF VARIABLES NAMES AND INSTANTIATION OF MOTORS AND SENSORS ###
# This is the system layout definition (which objects and which ev3s they're connected to #

input_sensor  = ColorSensor('input_sensor', 'ev3A')
color_sensor  = ColorSensor('color_sensor', 'ev3A')
rfid_sensor  = ColorSensor('rfid_sensor', 'ev3B')
rfid_1  = ColorSensor('rfid_1', 'ev3B')
rfid_2  = ColorSensor('rfid_2', 'ev3B')
rfid_scraps  = ColorSensor('rfid_scraps', 'ev3B')

conveyor = Motor('conveyor', 'ev3A')
color_select = Motor('color_select', 'ev3A')
rfid_select = Motor('rfid_select', 'ev3A')
rfid_conveyor = Motor('rfid_conveyor', 'ev3A')

### Publishing ev3_config messages to configure ev3s ###
# You have to modify these for the system definition #

pub_config()

### FAKE ORDER RECEPTION ###

order1 = {1:('red','blue')}
order2 = {1:('blue','blue')}
order0 = {}

orders.append(order1)
orders.append(order2)
print (orders)

### MAIN LOOP ###

# bring the color selecter to right position (angle = 0)
color_select.abs(angle1=ColorAngle0+1, angle2=ColorAngle0)
rfid_select.abs(RfidAngle0+1, RfidAngle0)

while not condition:
    sleep(3)

while condition:

    # if there's ball waiting at upstream buffer and the "color-check" station is idle let the ball enter the system
    if input_sensor.output['value'] in gcolors and color_sensor_available:

        # updates counter_in and publishes on-change
        counter_in += 1
        pub('selezionatori/counters', 'int', 'counter_in', counter_in)

        conveyor.rel1verse(speed=ConveyorSpeed, angle=ConveyorAngle)
        #color_sensor_available = color-check station state: False = working, True = idle, ? = failure
        color_sensor_available = False
        color_timer_start = time()
        pub('selezionatori/states', 'int', 'color_station_state', int(not color_sensor_available))
        sleep(4)
        if input_sensor.output['value'] not in gcolors:
            #fai entrare la nuova pallina, se questo if è falso significa che non ha tirato su la pallina ed è rimasta lì
            pass

    # if "color-check" station is in state "working"
    if not color_sensor_available:    

        color_timer = time() - color_timer_start

        # if there's a ball waiting at color check or timer exceeded the limit
        if color_sensor.output['value'] in gcolors or color_timer >= ColorTimerLimit:

            # updates color_data (the color seen by the color sensor) and publishes it on-change
            color_data = color_sensor.output['value']
            pub('selezionatori/counters', 'str', 'color_data', color_data)            

            # updates the counter_color and publishes on-change
            counter_color += 1
            pub('selezionatori/counters', 'int', 'counter_color', counter_color)

            # checks if the color seen is in the fisrt two orders (the ones waiting at the parts output channels)
            for order in orders[:2]:
                for element in order.values():
                    # if color seen is present in the first two orders, let the ball pass
                    if color_sensor.output['value'] == element[0]:
                        color_select.abs(angle1=ColorAngleV, angle2=ColorAngle0)
                        color_to_rfid_timer_start = time()
                        color_to_rfid_timer = -1.0
                        print(color_to_rfid_timer)
                        # sets the color-check station state to idle
                        color_sensor_available = True
                        pub('selezionatori/states', 'int', 'color_station_state', int(not color_sensor_available))
                        # exits the for cycle (ball has already been discharged)
                        break
                if color_sensor_available == True:
                    # exits the for cycle (ball has already been discharged)
                    break

            # color_sensor_available = False means ball is still there (wasn't in the orders) or color-check timeout happened
            # send the ball to scraps
            if color_sensor_available == False:
                color_select.abs(angle1=ColorAngleX, angle2=ColorAngle0)

                # updates the color_scrap_counter and publishes on-change
                color_scrap_count += 1
                pub('selezionatori/counters', 'int', 'color_scrap_count', color_scrap_count)
                # note that if ball didn't enter the system you're counting one scrap more than the actual scraps
                # anyway also the counter-in will show this same one extra ball, so the equality counter_in = counter_out will be verified

                # updates the counter_out and publishes on-change
                counter_out += 1
                pub('selezionatori/counters', 'int', 'counter_out', counter_out)

                # sets the color-check station state to idle
                color_sensor_available = True
                pub('selezionatori/states', 'int', 'color_station_state', int(not color_sensor_available))

    if color_to_rfid_timer != 0.0:
        color_to_rfid_timer = time() - color_to_rfid_timer_start
        if color_to_rfid_timer > ColorToRfidTimerLimit:
            print('Error! No ball or unrecognised ball has reached the RFID sensor. Beginning discharging procedure... ')
            rfid_conveyor.rel1verse(speed=RfidConveyorSpeed, angle = RfidConveyorAngle)
            color_to_rfid_timer = 0.0
            sleep(4)
            print('Discharging procedure completed')

            #updates the counter_out and publishes it on change
            counter_out += 1
            pub('selezionatori/counters', 'int', 'counter_out', counter_out)

            #updates the rfid_scrap_count and publishes it on-change
            rfid_scrap_count += 1
            pub('selezionatori/counters', 'int', 'rfid_scrap_count', rfid_scrap_count)

    # if "rfid-check" station state is "working" (there is a ball waiting at it)
    if rfid_present:

        color_to_rfid_timer = 0.0

        # updates rfid_data (the rfid tag read by the sensor) and publishes it on-change
        rfid_data = rfid_sensor.output['value']
        pub('selezionatori/counters', 'str', 'rfid_data', rfid_data)

        # updates counter_rfid and publishes it on-change
        counter_rfid += 1
        pub('selezionatori/counters', 'int', 'counter_rfid', counter_rfid)

        rfid_timer_start = time()

        # j reinitialization
        j = 0
        # rfid check for first order  <-- FIFO logic for completing the orders!
        order = orders[0]

        # if rfid value in first order, route ball to first bucket
        for key, element in order.items():
            if rfid_sensor.output['value'] == element[1]:

                processed_order = 1
                
                #updates the order1_counter and publishes it on-change
                order1_count += 1
                pub('selezionatori/counters', 'int', 'order1_count', order1_count)
          
                #updates the counter_out and publishes it on change
                counter_out += 1
                pub('selezionatori/counters', 'int', 'counter_out', counter_out)

                rfid_timer = 0.0
                rfid_conveyor.forever(RfidConveyorSpeed)
                # waits for the ball to be in the right channel for being discharged to order 1
                while rfid_1_empty and rfid_timer < RfidTimerLimit:
                    rfid_timer = time() - rfid_timer_start
                    sleep(0.1)
                rfid_1_empty = True    
                rfid_conveyor.stop()
                rfid_select.abs(RfidAngle, RfidAngle0)

                # delete the processed element of the order
                del order[key]
                print('Order '+str(processed_order)+' now is:')
                print(order)

                # j = 1 indicates that the ball has already been discharged
                j = 1
                # sets the "rfid-check" station state to idle
                rfid_present = False
                pub('selezionatori/states', 'int', 'rfid_station_state', int(rfid_present))
                # no need to check if the ball is in other elements for order 1 
                break

        # if order is empty it is finished, delete it
        if order == {}:
            print('Order finished, deleting it.')
            del orders[0]
            if len(orders) == 1:
                orders.append({})
            # updates teh order1_finished variable and publishes it on change
            order1_finished = True
            pub('selezionatori/counters', 'bool', 'order1_finished', order1_finished)

        # if j = 0 means the ball is still there, check if it is in the second order
        if j == 0:
            order = orders[1]

            processed_order = 2

            # if rfid value in second order, route ball to second bucket
            for key, element in order.items():
                if rfid_sensor.output['value'] == element[1]:
                
                    #updates the order1_counter and publishes it on-change
                    order2_count += 1
                    pub('selezionatori/counters', 'int', 'order2_count', order2_count)

                    #updates the counter_out and publishes it on change
                    counter_out += 1
                    pub('selezionatori/counters', 'int', 'counter_out', counter_out)

                    rfid_conveyor.forever(RfidConveyorSpeed)
                    rfid_timer = 0.0                    
                    # waits for the ball to be in the right channel for being discharged to order 2
                    while rfid_2_empty and rfid_timer < RfidTimerLimit:
                        rfid_timer = time() - rfid_timer_start
                        sleep(0.1)
                    rfid_2_empty = True
                    rfid_conveyor.stop()
                    rfid_select.abs(RfidAngle, RfidAngle0)

                    # delete the processed element of the order
                    del order[key]
                    print('Order '+str(processed_order)+' now is:')
                    print(order)

                    # j = 1 indicates that the ball has already been discharged
                    j = 1
                    # sets the "rfid-check" station state to idle
                    rfid_present = False
                    pub('selezionatori/states', 'int', 'rfid_station_state', int(rfid_present))
                    # no need to check if the ball is in other elements for order 1 
                    break

            # if order is empty it is finished, delete it        
            if order == {}:
                print('Order finished, deleting it.')
                del orders[1]
                if len(orders) == 1:
                    orders.append({})
                # updates the order1_finished variable and publishes it on change
                order2_finished = True
                pub('selezionatori/counters', 'bool', 'order2_finished', order2_finished)

        # if j = 0 means the ball is still there (or wasn't recognised or it never entered), it was not present nor in order 1 nor in 2, it is discarded
        if j == 0:

            processed_order = 0

            #updates the rfid_scrap_count and publishes it on-change
            rfid_scrap_count += 1
            pub('selezionatori/counters', 'int', 'rfid_scrap_count', rfid_scrap_count)

            #updates the counter_out and publishes it on change
            counter_out += 1
            pub('selezionatori/counters', 'int', 'counter_out', counter_out)

            rfid_conveyor.forever(RfidConveyorSpeed)
            rfid_timer = 0.0
            # waits for the ball to be in the right channel for being discharged to scraps
            while rfid_scraps_empty and rfid_timer < RfidTimerLimit:
                rfid_timer = time() - rfid_timer_start
                sleep(0.1)
            rfid_scraps_empty = True

            rfid_conveyor.stop()
            if rfid_timer < RfidTimerLimit:
                rfid_select.abs(RfidAngle, RfidAngle0)
            # sets the "rfid-check" station state to idle
            rfid_present = False
            pub('selezionatori/states', 'int', 'rfid_station_state', int(rfid_present))

        processed_order = -1    

    sleep(0.5)

print('Ending listening. Closing script.')
client.loop_stop()
