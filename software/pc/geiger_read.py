import serial
import sys
import math
BAUDRATE = 115200
PARITY = False

#Configures serial port
def configure_serial(serial_port):
    return serial.Serial(
        port=serial_port,
        baudrate=BAUDRATE,
        parity=serial.PARITY_EVEN if PARITY else serial.PARITY_NONE,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.EIGHTBITS,
        timeout=0.1
    )

def parse_event(x):
    return int(x[1:4].strip()),int(x[5:8].strip())


class RingBuffer():
    def __init__(self, size=10):
        self.size = size
        self.ring = [None]*size
        self.full = False
        self.pointer = 0

    def average(self):
        if self.full:
            return sum(self.ring)/self.size
        else:
            if self.pointer != 0:
                return sum(self.ring[:self.pointer])/self.pointer
            return None

    def average_of(self, n):
        if not self.full and self.pointer < n:
            #Not enough elements in buffer
            return None
        return sum([self.get_nth_latest(i) for i in xrange(n)])/float(n)

    def get_nth_latest(self, n):
        return self.ring[(self.pointer-n-1)%self.size]

    def insert(self, element):
        self.ring[self.pointer] = element
        self.pointer = (self.pointer+1)%self.size
        if self.pointer == 0:
            self.full = True

    def resize(self, new_size):
        if new_size == self.size:
            return None
        if new_size > self.size:
            self.full = False
            self.ring.extend([None]*(new_size-self.size))
            self.size = new_size
        else:
            new_ring = [self.get_nth_latest(i) for i in xrange(new_size)]
            self.ring = new_ring
            self.size = new_size
            if self.pointer >= new_size:
                self.pointer = 0
                self.full = True

    def zero(self):
        self.ring = [None]*self.size
        self.pointer = 0
        self.full = False

#chi squared distribution inverse CDF values for estimating the 95% confidence
#interval, index i is value for 2*i degrees of freedom
inversechi2_95s = [7.37776, 11.1433, 14.4494, 17.5345, 20.4832, 23.3367, 26.1189, \
28.8454, 31.5264, 34.1696, 36.7807, 39.3641, 41.9232, 44.4608, \
46.9792, 49.4804, 51.966, 54.4373, 56.8955, 59.3417, 61.7768, \
64.2015, 66.6165, 69.0226, 71.4202, 73.8099, 76.192, 78.5672, \
80.9356, 83.2977, 85.6537, 88.0041, 90.3489, 92.6885, 95.0232, \
97.3531, 99.6783, 101.999, 104.316, 106.629, 108.937, 111.242, \
113.544, 115.841, 118.136, 120.427, 122.715, 125., 127.282, 129.561, \
131.838, 134.111, 136.382, 138.651, 140.917, 143.18, 145.441, 147.7, \
149.957, 152.211, 154.464, 156.714, 158.962, 161.209, 163.453, \
165.696, 167.936, 170.175, 172.412, 174.648, 176.882, 179.114, \
181.344, 183.573, 185.8, 188.026, 190.251, 192.474, 194.695, 196.915, \
199.134, 201.351, 203.567, 205.782, 207.995, 210.208, 212.419, \
214.628, 216.837, 219.044, 221.251, 223.456, 225.66, 227.863, \
230.064, 232.265, 234.465, 236.664, 238.861, 241.058]

#Same as last one, but in different direction. Used for estimating upper limit
inversechi2_95b = [0.0506356, 0.484419, 1.23734, 2.17973, 3.24697, 4.40379, 5.62873, \
6.90766, 8.23075, 9.59078, 10.9823, 12.4012, 13.8439, 15.3079, \
16.7908, 18.2908, 19.8063, 21.3359, 22.8785, 24.433, 25.9987, \
27.5746, 29.1601, 30.7545, 32.3574, 33.9681, 35.5863, 37.2116, \
38.8435, 40.4817, 42.126, 43.776, 45.4314, 47.092, 48.7576, 50.4279, \
52.1028, 53.7821, 55.4656, 57.1532, 58.8446, 60.5398, 62.2386, \
63.9409, 65.6466, 67.3556, 69.0677, 70.7828, 72.5009, 74.2219, \
75.9457, 77.6722, 79.4013, 81.1329, 82.8671, 84.6036, 86.3425, \
88.0837, 89.8271, 91.5726, 93.3203, 95.0701, 96.8219, 98.5756, \
100.331, 102.089, 103.848, 105.609, 107.372, 109.137, 110.903, \
112.671, 114.441, 116.212, 117.985, 119.759, 121.534, 123.312, \
125.09, 126.87, 128.651, 130.434, 132.218, 134.003, 135.79, 137.578, \
139.367, 141.157, 142.949, 144.741, 146.535, 148.33, 150.126, \
151.923, 153.721, 155.521, 157.321, 159.122, 160.925, 162.728]

if __name__ == "__main__":

    if len(sys.argv)!=2:
        print "Give serial port address as a command line argument."
        exit()
    try:
        ser = configure_serial(sys.argv[1])
        if not ser.isOpen():
            raise Exception
    except:
        print 'Opening serial port {} failed.'.format(sys.argv[1])
        raise
        exit()

    ring_size = 100
    event_ring = RingBuffer(ring_size)
    ticks_last_event = 0
    #One tick of event counter time in seconds
    tick = 1024/float(16*10**6)
    #Enable sending counts over USB
    ser.write('s')
    last_ticks = RingBuffer(4)
    events = 0

    #Probability of event to zero buffer
    p = 0.001
    #Multiple of mean time the event needs to be to have probability of
    #p happening
    zero1_p = -math.log(p)
    zero2_p = -math.log(p**(1.0/3))
    zero3_p = -math.log(1-p**(1.0/4))
    try:
        while True:
            try:
                x = ser.readline()
            except serial.serialutil.SerialException:
                continue
            except OSError:
                continue
            #If activity decreases significantly zero buffer. This has chance
            #of approximately p happening if the radioactivity hasn't changed.
            #Check last event against average
            if events > 0 and ticks_last_event*tick > zero1_p*2*events*event_ring.average()/inversechi2_95b[events-1]:
                events = 0
                #print 'zero1'
                cpm = 0
                event_ring.zero()
                last_ticks.zero()
            #Check three last events
            elif events > 2 and last_ticks.average_of(3) > zero2_p*2*events*event_ring.average()/inversechi2_95b[events-1]:
                #print 'zero2'
                events = 0
                cpm = 0
                event_ring.zero()
                last_ticks.zero()
            if x[0]=='c':
                e,t = parse_event(x)
                if e != 0:
                    for i in xrange(e):
                        events += 1
                        events = min(ring_size, events)
                        ticks_last_event += t/float(e)
                        event_ring.insert(ticks_last_event*tick)
                        last_ticks.insert(ticks_last_event*tick)
                        cpm = 60/(event_ring.average())
                        if cpm < 100 and last_ticks.average_of(4)!= None:
                            if last_ticks.average_of(4) < zero3_p*2*events*event_ring.average()/inversechi2_95s[events-1]:
                                #print 'zero3'
                                events = 1
                                event_ring.zero()
                                last_ticks.zero()
                                event_ring.insert(ticks_last_event*tick)
                                last_ticks.insert(ticks_last_event*tick)
                        print "CPM: {0:.2f}".format(cpm)
                        ticks_last_event = 0
                   # if events>1 and events%10 == 0:
                   #     #3 seconds worth of samples
                   #     #minimum of 10 samples and maximum of 100
                   #     new_size = min(max(10,int(5/event_ring.average())),100)
                   #     print 'resize {}'.format(new_size-event_ring.size)
                   #     event_ring.resize(new_size)
                   #     events = min(events,new_size)
                   #     print 'events',events
                else:
                    ticks_last_event += 256
            else:
                print x
    except KeyboardInterrupt:
        ser.write('S')
        ser.close()
