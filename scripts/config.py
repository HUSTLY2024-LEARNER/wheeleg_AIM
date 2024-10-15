import json

class Config:
    def __init__(self, json_file_path, section):
        self.load_from_json(json_file_path, section)

    def load_from_json(self, json_file_path, section):
        try:
            with open(json_file_path, 'r') as file:
                data = json.load(file)
                config_data = data.get(section, {})
                for key, value in config_data.items():
                    setattr(self, key, value)
        except Exception as e:
            print(f"Error loading JSON config: {e}")