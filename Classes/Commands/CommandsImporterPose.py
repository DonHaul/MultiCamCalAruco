import CommandsImporter
import sys,os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))+"/../..")
from libs import *
import rospy

class CommandsImporterPose(CommandsImporter.CommandsImporter):
    def __init__(self,invoker,posepipeline):
        invoker.register("R",CalculateRotations(posepipeline.posescalculator))
        invoker.register("T",CalculateTranslations(posepipeline))
        invoker.register("help",HelpCommand(posepipeline))


class CalculateRotations(CommandsImporter.Command):
    
    def __init__(self, posescalculator):
        self._posescalc = posescalculator

    def execute(self,*args):
        rr = self._posescalc.CalcRthenStartT()
        visu.ViewRefs(rr)

class CalculateTranslations(CommandsImporter.Command):
    
    def __init__(self, state ):
        self._state = state

    def execute(self,*args):
        self._state.posescalculator.CalcT()
        rospy.signal_shutdown("Successful T")
        self._state.stop_threads=True

class HelpCommand(CommandsImporter.Command):
    """A Command object, which implemets the ICommand interface"""

    def __init__(self, light):
        self._light = light

    def execute(self,*args):
        self._light.help(args)