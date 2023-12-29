def parse_robotics_data(filename):
    with open(filename, "r") as file:
        file_content = file.read()

    # Splitting the content by 'Iteration'
    iterations = file_content.strip().split("Iteration ")[1:]

    data_dict = {}

    for iteration in iterations:
        # Splitting each iteration into its components
        lines = iteration.split("\n")
        iteration_number = int(lines[0].strip())
        plug_index = lines.index("plug_link: ") + 1
        socket_index = lines.index("socket_link: ") + 1

        # Extracting and converting the plug_link matrix
        plug_matrix = [list(map(float, line.split())) for line in lines[plug_index : socket_index - 1]]
        # Extracting and converting the socket_link matrix
        socket_matrix = [list(map(float, line.split())) for line in lines[socket_index : socket_index + 4]]

        # Storing in the dictionary
        data_dict[iteration_number] = {"plug_link": plug_matrix, "socket_link": socket_matrix}

    return data_dict


if __name__ == "__main__":
    filename = "./test_data/log.txt"
    parsed_data = parse_robotics_data(filename)

    # Print or process the parsed data as needed
    print(parsed_data)
