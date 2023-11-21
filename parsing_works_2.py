import xml.etree.ElementTree as ET

xml_file_path = r"D:\Master HCI\2. Semester\Projekt\behavior_tree_dataset\behaviortreecpp\CARVE-ROBMOSYS_BTCompiler_carve_use_case_3.xml"

# Read XML content from the file
with open(xml_file_path, 'r', encoding='utf-8') as file:
    xml_content = file.read()

root = ET.fromstring(xml_content)

def extract_action_ids(element):
    action_ids = set()

    # Define a recursive function to traverse the XML tree
    def traverse(element):
        if element.tag.endswith("Action") and "ID" in element.attrib:
            action_ids.add(element.attrib["ID"])
        for child in element:
            traverse(child)

    # Start traversing from the provided element
    traverse(element)

    return action_ids

# Extract action IDs from the BehaviorTree and TreeNodesModel elements
behavior_tree_action_ids = extract_action_ids(root.find(".//BehaviorTree"))
tree_nodes_model_action_ids = extract_action_ids(root.find(".//TreeNodesModel"))

# Combine and print the result
all_action_ids = behavior_tree_action_ids.union(tree_nodes_model_action_ids)
print("All Action IDs:")
for action_id in all_action_ids:
    print(action_id)
