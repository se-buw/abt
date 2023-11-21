import xml.etree.ElementTree as ET

# Specify the path to your XML file
xml_file_path = r"D:\Master HCI\2. Semester\Projekt\behavior_tree_dataset\behaviortreecpp\vislab-tecnico-lisboa_vizzy_playground_security.xml"

# Parse the XML file
tree = ET.parse(xml_file_path)
root = tree.getroot()

def extract_action_ids_and_count(element):
    action_ids = set()

    # Define a recursive function to traverse the XML tree
    def traverse(element):
        nonlocal action_ids
        if element.tag.endswith("Action") and "ID" in element.attrib:
            action_ids.add(element.attrib["ID"])
        for child in element:
            traverse(child)

    # Start traversing from the provided element
    traverse(element)

    return action_ids, len(action_ids)

# Extract action IDs and count from the root element
action_ids, action_count = extract_action_ids_and_count(root)

# Print the result
print("Action IDs:")
for action_id in action_ids:
    print(action_id)

print("\nTotal number of actions:", action_count)
