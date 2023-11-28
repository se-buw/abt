import xml.etree.ElementTree as ET

# Specify the path to your XML file
xml_file_path = r"D:\Master HCI\2. Semester\Projekt\behavior_tree_dataset\behaviortreecpp\vislab-tecnico-lisboa_vizzy_playground_nothing.xml"

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

def generate_tree_description(element, parent_condition=None):
    description = ""

    # Define a recursive function to generate descriptions
    def describe(element, indent=""):
        nonlocal description
        if element.tag.endswith("Action") and "ID" in element.attrib:
            if parent_condition:
                description += f"{indent}If the condition '{parent_condition}' is fulfilled, perform action: '{element.attrib['ID']}'\n"
            else:
                description += f"{indent}Perform action: '{element.attrib['ID']}'\n"
        elif element.tag.endswith("Sequence"):
            description += f"{indent}Execute the following actions sequentially:\n"
        elif element.tag.endswith("Fallback"):
            description += f"{indent}Try the following actions in order until one succeeds:\n"
        elif element.tag.endswith("Parallel"):
            description += f"{indent}Execute the following actions in parallel:\n"
        elif element.tag.endswith("Condition"):
            condition_id = element.attrib['ID']
            description += f"{indent}If the condition '{condition_id}' is fulfilled, these actions are performed:\n"
            for child in element:
                describe(child, indent + "  ")
        elif element.tag.endswith("SubTree"):
            description += f"{indent}Execute SubTree: '{element.attrib['ID']}'\n"

        # Recursively describe children
        for child in element:
            describe(child, indent + "  ")

    # Start describing from the root
    describe(element)

    return description

# Extract action IDs and count from the root element
action_ids, action_count = extract_action_ids_and_count(root)

# Print the result
print("Action IDs:")
for action_id in action_ids:
    print(action_id)

print("\nTotal number of actions:", action_count)

# Generate and print natural language description of the behavior tree
tree_description = generate_tree_description(root)
print("\nBehavior Tree Description:")
print(tree_description)
