import xml.etree.ElementTree as ET
import sys

def process_node(node, node_number, indent=0):
    attributes_str = ', '.join([f"{key}='{value}'" for key, value in node.attrib.items()])
    #node_type = get_node_type(node.tag)
    output = "  " * indent + f"{node_number}. {node.tag} node"
    
    num_attributes = len(node.attrib)
    if num_attributes == 1:
        output += f" has an attribute: {{{attributes_str}}}"
    elif num_attributes > 1:
        output += f" has attributes: {{{attributes_str}}}"

    num_children = len(node)
    if num_children > 1:
        if num_attributes > 0:
            output += f" and has {num_children} children nodes"
        else:
            output += f" has {num_children} children nodes"
    elif num_children == 1:
        if num_attributes > 0:
            output += f" and has {num_children} children node"
        else:
            output += f" has {num_children} children node"
    else:
        output += " and has no children nodes"

    # Process children recursively
    for i, child in enumerate(node, start=1):
        output += "\n" + process_node(child, f"{node_number}.{i}", indent + 1)

    return output

def get_node_type(tag):
    node_types = {
        'root': ("Root", "Represents the root of the behavior tree"),
        'BehaviorTree': ("BehaviorTree", "Represents a node that defines the behavior tree in XML"),
        'Repeat': ("Repeat", "Represents a node that repeats the execution of its children for a specified number of cycles"),
        'Sequence': ("Sequence", "Represents a node that executes its children sequentially until one fails or all succeed"),
        'SetBlackboard': ("SetBlackboard", "Represents a node that sets a value in the blackboard"),
        'SubTree': ("SubTree", "Represents a reference to another behavior tree"),
        'Nav2Client': ("Nav2Client", "Represents a node that interacts with a navigation client. It has a goal attribute with the placeholder 'target' indicating that it will use the target specified when this behavior tree is called."),
        'Selector': ("Selector", "Represents a node that selects and executes one of its children based on their success or failure."),
        'Parallel': ("Parallel", "Represents a node that executes all its children simultaneously."),
        'Fallback': ("Fallback", "Represents a node that executes its children sequentially until one succeeds."),
        'Action': ("Action", "Represents a leaf node that performs a specific action."),
        'Condition': ("Condition", "Represents a leaf node that checks a condition."),
        'Decorator': ("Decorator", "Represents a node that can modify the behavior of its child nodes."),
    }
    return node_types.get(tag, (None, None))

def main():
    # Parse the XML file
    tree = ET.parse('Adlink-ROS_BT_ros2_bt_nav_mememan.xml')
    root = tree.getroot()

    # Save the output to a file
    output_file_path = 'output.txt'
    with open(output_file_path, 'w') as output_file:
        sys.stdout = output_file

        # Print node type information
        print(f"Types of nodes:")
        processed_types = set()
        for node in root.iter():
            tag = node.tag
            node_type, node_info = get_node_type(tag)
            if node_type and node_type not in processed_types:
                processed_types.add(node_type)
                print(f"{node_type.capitalize()} node: {node_info}")

        print("\nThe tree has the following structure:")

        # Process the root node
        attributes_str = ', '.join([f"{key}='{value}'" for key, value in root.attrib.items()])
        output = f"1. {root.tag} node"
        if len(root.attrib) == 1:
            output += f" has an attribute: {{{attributes_str}}}"
        elif len(root.attrib) > 1:
            output += f" has attributes: {{{attributes_str}}}"

        num_children = len(root)
        if num_children > 1 and len(root.attrib) > 0:
            output += f" and has {num_children} children nodes"
        elif num_children == 1 and len(root.attrib) > 0:
            output += f" and has {num_children} children node"
        elif num_children > 1:
            output += f" has {num_children} children nodes"
        elif num_children == 1:
            output += f" has {num_children} children node"
        else:
            output += " and has no children nodes"

        # Print the result
        print(output)
        for i, child in enumerate(root, start=1):
            print(process_node(child, f"1.{i}", indent=1))
        sys.stdout = sys.__stdout__
    print(f"Output saved to {output_file_path}")

if __name__ == "__main__":
    main()
