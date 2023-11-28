import xml.etree.ElementTree as ET
import py_trees

def parse_behavior_tree_xml(node, element):
    if isinstance(node, py_trees.trees.BehaviourTree):
        tree_element = ET.SubElement(element, 'BehaviorTree')
        root_node = node.root
        parse_node_xml(root_node, tree_element)
    else:
        raise ValueError("Invalid input. Expected a py_trees.trees.BehaviourTree.")

def parse_node_xml(node, parent_element):
    node_element = ET.SubElement(parent_element, node.name)
    
    if isinstance(node, py_trees.behaviours.behaviour.Behaviour):
        ET.SubElement(node_element, 'Type').text = type(node).__name__
    elif isinstance(node, py_trees.composites.composite.Composite):
        ET.SubElement(node_element, 'Type').text = type(node).__name__
        for child_node in node.children:
            parse_node_xml(child_node, node_element)
    else:
        raise ValueError(f"Unsupported node type: {type(node)}")

def main():
    # Create a simple behavior tree
    root = py_trees.composites.Selector(name="Root")
    idle = py_trees.behaviours.Success(name="Idle")
    active = py_trees.behaviours.Failure(name="Active")
    root.add_children([idle, active])
    behavior_tree = py_trees.trees.BehaviourTree(root)

    # Parse the behavior tree and create an XML representation
    root_element = ET.Element('Root')
    parse_behavior_tree_xml(behavior_tree, root_element)

    # Create an ElementTree object and write to an XML file
    tree = ET.ElementTree(root_element)
    tree.write("parsed_behavior_tree.xml")

if __name__ == '__main__':
    main()
