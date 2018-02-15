from simtestutil import SimTestCase, SimTestLoader
import pysimulavr

class TestCase(SimTestCase):
  
  DELAY = 5000 # run 5 microseconds

  ain1_pin = {
    "at90s4433":  "D7",
    "at90s8515":  "B3",
    "at90can32":  "E3",
    "attiny25":   "B1",
    "atmega644":  "B3",
    "atmega16":   "B3",
    "atmega128":  "E3",
    "atmega48":   "D7",
    "attiny2313": "B1",
    "atmega8":    "D7",
  }

  def test_00(self):
    """check state of analog comparator"""
    self.assertDevice()
    self.assertStartTime()
    # create analog pin as injector and connect to AIN1
    apin = pysimulavr.Pin(1.0) # below bandgap voltage level
    net = pysimulavr.Net()
    net.Add(apin)
    net.Add(self.dev.GetPin(self.ain1_pin[self.processorName])) # connect to AIN1
    if self.processorName == "at90s8515":
      # create analog pin as injector and connect to AIN0, because at90s8515 has no bandgap source to AIN0
      rpin = pysimulavr.Pin(1.5)
      net2 = pysimulavr.Net()
      net2.Add(rpin)
      net2.Add(self.dev.GetPin("B2")) # connect to AIN0
    # get a output pin
    opin = self.dev.GetPin("B0")
    # skip initialisation
    self.assertInitDone()
    # check, that we are not in idle loop ...
    self.assertEqual(self.sim.getByteByName(self.dev, "in_loop"), 0, "not in idle loop")
    # run till in idle loop
    self.sim.doRun(self.sim.getCurrentTime() + self.DELAY)
    # check, that we are now in idle loop ...
    self.assertEqual(self.sim.getByteByName(self.dev, "in_loop"), 1, "in idle loop")
    # check output state
    self.assertEqual(opin.toChar(), "H", "output value wrong: got=%s, exp=H" % opin.toChar())
    # set voltage on apin to 2.0V
    apin.SetAnalogValue(2.0)
    # run
    self.sim.doRun(self.sim.getCurrentTime() + self.DELAY)
    # check output state
    self.assertEqual(opin.toChar(), "L", "output value wrong: got=%s, exp=L" % opin.toChar())

if __name__ == '__main__':
  
  from unittest import TextTestRunner
  tests = SimTestLoader("anacomp_at90s4433.elf").loadTestsFromTestCase(TestCase)
  TextTestRunner(verbosity = 2).run(tests)

# EOF
