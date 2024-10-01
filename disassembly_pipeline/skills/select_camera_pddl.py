import numpy as np
from robotblockset_python.transformations import *
from unified_planning.shortcuts import *
from disassembly_pipeline.skills.base_skill import BaseSkill, SkillExecutionResult
from action_prediction_interface.pddl_modules.base_pddl_skill import BaseSkillPDDLWrapper


class SelectCameraPDDLWrapper(BaseSkillPDDLWrapper):
    def __init__(self,
                 camera_names = ['eye_in_hand_camera', 'table_vision_camera']):

        name = 'select_camera'
        description = 'Select a camera from which to receive image input.'
        action_arg_names = {'camera': 'the camera name from which to receive images.'}
        self.camera_names = camera_names

        class SelectCameraSkill:
            def __init__(self):
                0
            def on_enter_pddl(self, **kwargs):
                print(kwargs)
                env_flags = kwargs['env_flags']
                camera_to_look_at = kwargs['camera']
                env_flags['looking_at_camera'] = camera_to_look_at
                return 0
            #    return result

        skill_object = SelectCameraSkill()
        
        super().__init__(name = name, description = description, action_arg_names = action_arg_names, skill_object = skill_object)

    def pddl_init(self, problem, pddl_to_world_obj_links):
        # Add camera object
        try:
            camera_type = UserType('camera')
        except:
            camera_type = problem.user_type('camera')

        for camera in self.camera_names:
            if not problem.has_object(camera):
                problem.add_object(camera, camera_type)

        # Add looking at camera fluent
        looking_at_fluent_name = 'looking_at'
        looking_at = problem.fluent(looking_at_fluent_name)

        # Hardcoded - set default value of looking at the first-defined camera
        initially_looking_at_camera_name = self.camera_names[0] # TODO improve. Query VLM current camera topic?
        camera_object = problem.object(initially_looking_at_camera_name)
        problem.set_initial_value(looking_at(camera_object), True)

        # Add select_camera operator
        if not problem.has_action(self.get_name()):
            select_camera_operator = InstantaneousAction(self.get_name(), camera = camera_type)
            camera_param = select_camera_operator.parameter('camera')
            c = Variable("c", camera_type) # create a "loop" variable
            select_camera_operator.add_precondition(Not(looking_at(camera_param)))

            select_camera_operator.add_effect(fluent = looking_at(c),
                                              forall = (c,),    
                                              condition = Not(Equals(c, camera_param)),    
                                              value = False)
            select_camera_operator.add_effect(looking_at(camera_param), True)

            problem.add_action(select_camera_operator)

        return problem, pddl_to_world_obj_links

    def pddl_loop(self, problem, pddl_to_world_obj_links):
        return problem, pddl_to_world_obj_links
