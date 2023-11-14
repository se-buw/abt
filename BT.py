import py_trees

# Define the behavior tree
root = Selector("Root")
sequence1 = Sequence("Sequence1")
sequence1.add_child(Action("Action1"))
sequence1.add_child(Action("Action2"))
root.add_child(sequence1)
root.add_child(Action("Action3"))

# Parse the behavior tree into Python code
def parse_behavior_tree(node):
    if isinstance(node, Action):
        return f'do_action("{node.name}")'
    elif isinstance(node, Sequence):
        children_code = [parse_behavior_tree(child) for child in node.children]
        return f'sequence([{", ".join(children_code)}])'
    elif isinstance(node, Selector):
        children_code = [parse_behavior_tree(child) for child in node.children]
        return f'selector([{", ".join(children_code)}])'
    else:
        raise ValueError("Invalid node type")

# Generate Python code for the behavior tree
behavior_tree_code = parse_behavior_tree(root)
print(behavior_tree_code)