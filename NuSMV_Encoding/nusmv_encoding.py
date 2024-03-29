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

def collect_unique_actions(tree_root1, tree_root2):
    
    # Collect actions from the first tree
    bt_root1 = build_node_tree(tree_root1.find(".//BehaviorTree"))
    encoding = generate_encoding(bt_root1)
    states1 = list(encoding.keys())

    # Collect actions from the second tree
    bt_root2 = build_node_tree(tree_root2.find(".//BehaviorTree"))
    encoding = generate_encoding(bt_root2)
    states2 = list(encoding.keys())

    union_states = list(set(states1 + states2))

    # Return the union of actions from both trees
    return states1, states2, union_states

def write_unique_actions(smv_content, unique_actions):
    
    smv_content = "-- union of success variables for both behavior trees\n"
    smv_content += "\n".join([f"VAR {action}_success : boolean;" for action in unique_actions])
    return smv_content

def write_instances_to_compare(smv_content, states1, states2):

    smv_content = "\n\n-- instances of the two trees to compare"
    smv_content += f"\nVAR t1: tree1("
    smv_content += ', '.join([state + '_success' for state in states1])
    smv_content += ");" 

    smv_content += f"\nVAR t2: tree2("
    smv_content += ', '.join([state + '_success' for state in states2])
    smv_content += ");\n\n"

    return smv_content

def write_specifications():
    print("Write your LTL specifications (each specification separated by a line):")
    
    smv_content = []
    while True:
        line = input()
        if not line.strip():
            # If an empty line is encountered, finish input
            break
        smv_content.append(line)

    return "\n".join(smv_content)

def generate_nusmv_model(bt_root, tree_num):
    encoding = generate_encoding(bt_root)
    states = list(encoding.keys())

    smv_content = f"\n\nMODULE tree{tree_num}("
    smv_content += ', '.join([state + '_success' for state in states])
    smv_content += ")\n\n"

    smv_content += "\n".join([f"VAR {state} : boolean;" for state in states])
    smv_content += "\n\nVAR\n\tNode: {"
    smv_content += ', '.join([state + 'X' for state in states] + ['Done'])
    smv_content += "};\n\n"

    init_state = states[0] + "X" if states else "None"
    smv_content += f"INIT\n\tNode = {init_state};\n\n"

    smv_content += "ASSIGN\n"
    for state in states:
        smv_content += f"""  {state} := 
    case
      Node = {state}X : TRUE ;
      TRUE : FALSE;
    esac;\n\n"""

    smv_content += "  next(Node) :=\n    case\n"

    for state, successors in encoding.items():
        successors = ['_'.join(successor.split()) for successor in successors]
        if successors:
            #print(f"State: {state}, Successors: {successors}")
            if(successors[0]!=successors[1]):
                if(successors[0] == "Done"):
                    smv_content += f"      Node = {state}X : Done;\n"
                elif(successors[1] == "Done"):
                    smv_content += f"      Node = {state}X & next({successors[0]}_success) : {successors[0]}X;\n"
                    smv_content += f"      Node = {state}X & !next({successors[0]}_success) : Done;\n"
                else:
                    smv_content += f"      Node = {state}X & next({successors[0]}_success) : {successors[0]}X;\n"
                    smv_content += f"      Node = {state}X & !next({successors[0]}_success) : {successors[1]}X;\n"
            else:
                smv_content += f"      Node = {state}X : {successors[0]};\n"
        
    smv_content += "      TRUE : Done;\n    esac;"

    return smv_content
      

if __name__ == "__main__":
    input_file_tree1 = 'tree1.xml' 
    input_file_tree2 = 'tree2.xml'
    output_smv_path = 'NuSMV_Encoding.smv' 

    specifications = write_specifications()  # To write LTL specifications

    with open(output_smv_path, 'w') as file:
        file.write("MODULE main\n\n")

        # Read trees from files
        ### ----> Make sure one tree is a subset of another tree
        ### ----> Otherwise, we cannot compare action nodes or write specifications
        tree1 = read_behavior_tree_from_file(input_file_tree1)
        tree2 = read_behavior_tree_from_file(input_file_tree2)

        # Collect states and unique actions from both trees
        states1, states2, unique_actions = collect_unique_actions(tree1, tree2)
        
        # Write unique actions to the main module
        smv_content = write_unique_actions("", unique_actions)
        file.write(smv_content)

        # Write instances(states) of trees to compare
        smv_content = write_instances_to_compare("", states1, states2)
        file.write(smv_content)

        # Write LTLSPEC
        file.write("-- write specifications\n")
        file.write(specifications)

        # Write nusmv for tree1
        bt_root1 = build_node_tree(tree1.find(".//BehaviorTree"))
        nusmv_model1 = generate_nusmv_model(bt_root1, 1)
        file.write(nusmv_model1)

        # Write nusmv for tree2
        bt_root2 = build_node_tree(tree2.find(".//BehaviorTree"))
        nusmv_model2 = generate_nusmv_model(bt_root2, 2)
        file.write(nusmv_model2)

    # add both to the file
    print(f"NuSMV model written to {output_smv_path}")

