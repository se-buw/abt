class Node:
    def __init__(self, name):
        self.name = name

    def get_actions(self):
        return []

class Sequence(Node):
    def __init__(self, name):
        super().__init__(name)
        self.children = []

    def add_child(self, child):
        self.children.append(child)

    def get_actions(self):
        actions = []
        for child in self.children:
            actions.extend(child.get_actions())
        return actions

class Selector(Node):
    def __init__(self, name):
        super().__init__(name)
        self.children = []

    def add_child(self, child):
        self.children.append(child)

    def get_actions(self):
        actions = []
        for child in self.children:
            actions.extend(child.get_actions())
        return actions

class Action(Node):
    def __init__(self, name):
        super().__init__(name)

    def get_actions(self):
        return [self.name]
