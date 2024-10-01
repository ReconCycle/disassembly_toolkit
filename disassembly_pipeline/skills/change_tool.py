from disassembly_pipeline.skills.base_skill import BaseSkill, SkillExecutionResult


class ChangeTool(BaseSkill):
    def __init__(self):

        super().__init__()

    def on_enter_pddl(self, **kwargs) -> SkillExecutionResult:
        out = self.on_enter()
        return out

    def on_enter(self, robot, new_tool) -> SkillExecutionResult:
        result = SkillExecutionResult()
        return result

    def execute(self):
        pass

    def on_exit(self):
        pass


if __name__ == '__main__':
    0
