import xml.etree.ElementTree as ET

# Specify the path to your XML file
xml_file_path = r"D:\Master HCI\2. Semester\Projekt\behavior_tree_dataset\behaviortreecpp\MiRON-project_bundles_main.xml"

# Parse the XML file
tree = ET.parse(xml_file_path)
root = tree.getroot()

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

# Extract action IDs from the root element
action_ids = extract_action_ids(root)

# Print the result
print("Action IDs:")
for action_id in action_ids:
    print(action_id)
