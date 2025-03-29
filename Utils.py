def read_file(file_path):
    """Reads the content of a file and returns it."""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except FileNotFoundError:
        print(f"Error: The file {file_path} does not exist.")
        return None
    except Exception as e:
        print(f"An error occurred while reading the file {file_path}: {e}")
        return None
