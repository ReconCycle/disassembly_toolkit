from context_action_framework.msg import Detection

class DisassemblyObject(Detection):
    def __init__(self, detection:Detection, table_name:str, parent_frame:str, **kwargs):
        """ This is a simpler and more human-readable class than the Detection given by vision. 
        
        Args:
        -----
            - detection (Detection) : visual detection
            - table_name (str) : Name of table on which detection ocurred, if any.
            - parent_frame (str) : Name of the parent frame, in which the detection coordinates are given
        """
        super().__init__()

        self.detection = detection

        # For back-compatibility
        self.general_class = self.detection.label.name # General class such as HCA, SmokeDetector, etc.

        # What we need:
        # General class: hca/smoke_detector # Now called label.
        # Label_precise: 0 (qundis) # Now called label_precise
        # face: back/front, etc.
        # Transform Frame name: hca_back_0
        
        #self.general_class = self.detection.label.name # hca 
       

        self.general_class = self.detection.label.name 
        self.class_and_id = self.general_class + '_' + str(self.detection.id)
        #self.label_precise = "None" # Specific class such as HCA Kalo, HCA Qundis, etc.
        #self.table_name = table_name
        self.parent_frame = parent_frame

        self.confidence = self.detection.score
        # Quaternions defined as XYZW! Check and maybe fix!
        # TODO maybe : the table the item is on is important. Maybe add this attribute.

        self.tf_name = f"""{self.detection.tf_label.name}_{self.detection.id}_{self.parent_frame}"""

    def __str__(self):
        description = "{0} with confidence {1}, tf name {2} \n".format(self.general_class, round(self.confidence,3), self.tf_name)
        return description
    def get_class(self):
        return self.general_class

# TODO: find better place and finish
class SmokeDetector(DisassemblyObject):
    def __init__(self, disassembly_object):

        # We get the parent object as current input variable (disassembly_object). initialize the parent object with variables of current input variable
        super().__init__(**disassembly_object.__dict__) 

        self.perform_battery_check = False
        self.battery_rotation = None

        if 'front' in self.detection.tf_label.name:
            self.label_precise = 'fumonic'
        elif 'back' in self.detection.tf_label.name:
            self.label_precise = 'hekatron'
            self.perform_battery_check = True

        #self.tf_name = detection.tf_label.name + '_%d_%s'%(detection.id, detection.parent_frame)

    def set_cnc_params(self, cnc_function, additional_params = None):
        self.cnc_function = cnc_function
        self.cnc_params = additional_params

    # function to set which chuck should be used
    def set_chuck(self, chuck):
        self.chuck = chuck

    def set_detailed_class(self, detailed_class):
        self.label_precise = label_precise

class Hekatron(SmokeDetector):
    def __init__(self):
        super().__init__("Hekatron")
        self.perform_battery_check = True

class Fumonic(SmokeDetector):
    def __init__(self):
        super().__init__("Fumonic")