import xml.etree.ElementTree as ET

class Node:
    def __init__(self, name, node_type, parent=None, children=None):
        self.name = name
        self.node_type = node_type
        self.parent = parent
        self.children = children if children else []

    def add_child(self, child):
        self.children.append(child)

def read_behavior_tree_from_file(file_path):
    tree = ET.parse(file_path)
    return tree.getroot()

def build_node_tree(element, parent=None):
    node_type = element.tag
    node_name = element.get("name", None)
    node = Node(node_name, node_type, parent)
    for child in element:
        child_node = build_node_tree(child, node)
        node.children.append(child_node)
    return node

def find_successors(node, parent_node):
    success_successor = "Done"
    failure_successor = "Done"

    if parent_node.node_type == "Sequence":
        action_index = parent_node.children.index(node)
        # Successor for success scenario
        if action_index < len(parent_node.children) - 1:
            success_successor = find_first_action(parent_node.children[action_index + 1])
        else:
            success_successor = find_successor_upwards(node, parent_node, success=True)

        # Failure scenario: navigate upwards to find the next selector sibling
        failure_successor = find_successor_upwards(node, parent_node, success=False)

    elif parent_node.node_type == "Selector":
        index = parent_node.children.index(node)
        # Successor for failure scenario
        if index < len(parent_node.children) - 1:
            failure_successor = find_first_action(parent_node.children[index + 1])
        else:
            failure_successor = find_successor_upwards(node, parent_node, success=False)

        # Success scenario: navigate upwards to find the next sequence sibling
        success_successor = find_successor_upwards(node, parent_node, success=True)

    return [success_successor, failure_successor]

def find_successor_upwards(node, parent_node, success):
    while parent_node.parent and parent_node.parent.node_type != "Root":
        parent_node = parent_node.parent
        if parent_node.node_type == "Selector" and not success:
            selector_index = get_index_of_child_in_parent(node, parent_node)
            if selector_index < len(parent_node.children) - 1:
                return find_first_action(parent_node.children[selector_index + 1])
        elif parent_node.node_type == "Sequence" and success:
            sequence_index = get_index_of_child_in_parent(node, parent_node)
            if sequence_index < len(parent_node.children) - 1:
                return find_first_action(parent_node.children[sequence_index + 1])

    return "Done"

def get_index_of_child_in_parent(node, parent_node):
    for i, child in enumerate(parent_node.children):
        if is_same_node_or_contains(node, child):
            return i
    raise ValueError("Node not found in parent's children")

def is_same_node_or_contains(search_node, possible_parent):
    if search_node == possible_parent:
        return True
    if possible_parent.children:
        return any(is_same_node_or_contains(search_node, child) for child in possible_parent.children)
    return False

def find_first_action(node):
    if node.node_type == "Action":
        return node.name
    elif node.children:
        return find_first_action(node.children[0])
    return "Done"

def generate_encoding(tree_root):
    encoding = {}
    for child in tree_root.children:
        if child.node_type == "Action":
            successors = find_successors(child, tree_root)
            encoding[child.name.replace(" ", "_")] = successors
        elif child.node_type in ["Sequence", "Selector"]:
            encoding.update(generate_encoding(child))
    return encoding

def generate_nusmv_model(bt_root):
    encoding = generate_encoding(bt_root)
    states = list(encoding.keys())

    smv_content = "MODULE main\n\n"
    smv_content += "\n".join([f"VAR {state} : boolean;" for state in states])
    smv_content += "\n\nVAR\n\tNode: {"
    smv_content += ', '.join([state + 'X' for state in states] + ['Done'])
    smv_content += "};\n\n"

    init_state = states[0] + "X" if states else "None"
    smv_content += f"INIT\n\tNode = {init_state};\n\n"

    smv_content += "ASSIGN\n"
    for state in states:
        smv_content += f"""  next({state}) := 
    case
      next(Node) = {state}X : TRUE ;
      TRUE : FALSE;
    esac;\n\n"""

    smv_content += "  next(Node) :=\n    case\n"
    for state, successors in encoding.items():
        stateX = state + "X"
        if successors:
            # Modify how successors are processed
            successor_states = process_successors(successors)
            smv_content += f"      Node = {stateX} : {{{', '.join(successor_states)}}};\n"
        else:
            smv_content += f"      Node = {stateX} : Done;\n"
    smv_content += "      TRUE : Done;\n    esac;\n"

    return smv_content

def process_successors(successors):
    processed_successors = []
    for successor in successors:
        if successor != "Done":
            processed_successors.append(successor.replace(" ", "_") + 'X')
        else:
            processed_successors.append(successor)

    # Eliminate duplicate 'Done'
    if processed_successors == ["Done", "Done"]:
        return ["Done"]

    return processed_successors

if __name__ == "__main__":
    input_xml_path = 'robot.xml' 
    output_smv_path = 'NuSMV_Encoding_1.smv' 

    tree = read_behavior_tree_from_file(input_xml_path)
    bt_root = build_node_tree(tree.find(".//BehaviorTree"))
    nusmv_model = generate_nusmv_model(bt_root)

    with open(output_smv_path, 'w') as file:
        file.write(nusmv_model)
    print(f"NuSMV model written to {output_smv_path}")

