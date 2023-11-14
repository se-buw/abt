import markdown
from anytree import Node, RenderTree

class BehaviorNode:
    def __init__(self, name):
        self.name = name

    def execute(self):
        pass

    def to_markdown(self, indent=0):
        return f"{' ' * indent}- {self.name}\n"

    def to_tree(self, parent=None):
        return Node(self.name, parent=parent)

class Sequence(BehaviorNode):
    def __init__(self, name):
        super().__init__(name)
        self.children = []

    def add_child(self, child):
        self.children.append(child)

    def execute(self):
        print(f"Executing Sequence: {self.name}")
        for child in self.children:
            if not child.execute():
                return False
        return True

    def to_markdown(self, indent=0):
        result = super().to_markdown(indent)
        for child in self.children:
            result += child.to_markdown(indent + 2)
        return result

    def to_tree(self, parent=None):
        node = super().to_tree(parent)
        for child in self.children:
            child.to_tree(parent=node)
        return node

class Selector(BehaviorNode):
    def __init__(self, name):
        super().__init__(name)
        self.children = []

    def add_child(self, child):
        self.children.append(child)

    def execute(self):
        print(f"Executing Selector: {self.name}")
        for child in self.children:
            if child.execute():
                return True
        return False

    def to_markdown(self, indent=0):
        result = super().to_markdown(indent)
        for child in self.children:
            result += child.to_markdown(indent + 2)
        return result

    def to_tree(self, parent=None):
        node = super().to_tree(parent)
        for child in self.children:
            child.to_tree(parent=node)
        return node

class Action(BehaviorNode):
    def execute(self):
        print(f"Executing Action: {self.name}")
        return True

    def to_markdown(self, indent=0):
        return super().to_markdown(indent)

    def to_tree(self, parent=None):
        return super().to_tree(parent)

root = Sequence("Root")
root.add_child(Action("Walk to Door"))
root.add_child(Selector("Door Options"))
root.add_child(Action("Walk through Door"))
root.add_child(Action("Close Door"))

door_options = root.children[1]
door_options.add_child(Action("Open Door"))
door_options.add_child(Sequence("Unlock and Open Door"))
door_options.add_child(Action("Smash Door"))

unlock_open_sequence = door_options.children[1]
unlock_open_sequence.add_child(Action("Unlock Door"))
unlock_open_sequence.add_child(Action("Open Door"))

root.execute()

markdown_tree = root.to_markdown()
print(markdown_tree)

print("----------------------------------------")

html_tree = markdown.markdown(markdown_tree)
print(html_tree)

print("-----------------------------------------")

tree_node = root.to_tree()
for pre, _, node in RenderTree(tree_node):
    print(f"{pre}{node.name}")

