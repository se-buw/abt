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

def find_successors_in_selector(selector):
    successors = []
    for child in selector.children:
        if child.node_type == "Action":
            successors.append(child.name)
        elif child.node_type == "Sequence":
            if child.children:
                first_action = child.children[0]
                if first_action.node_type == "Action":
                    successors.append(first_action.name)
    return successors

def find_successors(action, parent_node):
    if parent_node.node_type == "Selector":
        return find_successors(parent_node, parent_node.parent)

    if parent_node.node_type == "Sequence":
        action_index = parent_node.children.index(action)
        if action_index < len(parent_node.children) - 1:
            next_node = parent_node.children[action_index + 1]
            if next_node.node_type == "Action":
                return [next_node.name]
            elif next_node.node_type == "Selector":
                return find_successors_in_selector(next_node)
        else:
            if parent_node.parent and parent_node.parent.node_type == "Selector":
                return find_successors(parent_node.parent, parent_node.parent.parent)

    return ["None"]

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
        if successors and successors[0] != "None":
            successor_states = [s.replace(" ", "_") + 'X' for s in successors]
            smv_content += f"      Node = {stateX} : {{{', '.join(successor_states)}}};\n"
        else:
            smv_content += f"      Node = {stateX} : Done;\n"
    smv_content += "      TRUE : Done;\n    esac;\n"

    return smv_content

if __name__ == "__main__":
    input_xml_path = 'tree2.xml' 
    output_smv_path = 'NuSMV_Encoding2.smv' 

    tree = read_behavior_tree_from_file(input_xml_path)
    bt_root = build_node_tree(tree.find(".//BehaviorTree"))
    nusmv_model = generate_nusmv_model(bt_root)

    with open(output_smv_path, 'w') as file:
        file.write(nusmv_model)
    print(f"NuSMV model written to {output_smv_path}")
