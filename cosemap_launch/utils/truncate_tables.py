import mariadb
import yaml
import os
import getpass

def main():
    
    pwd = os.getcwd()
    config_file = pwd + "/../config/db_utils.yaml"
    
    # parsing yaml file    
    with open(config_file, 'r') as stream:
        try:
            config = yaml.safe_load(stream)
            database_name = config["database_settings"]["database_name"]
            host = config["database_settings"]["host"]
            port = config["database_settings"]["port"]
        except yaml.YAMLError as e:
            print(e)
            
    user = input("Enter username: ")
    password = getpass.getpass("Enter password: ")
    
    # connect to MySQL/MariaDB server as root
    try:
        conn = mariadb.connect(
            user=user,
            password = password,
            host=host,
            port=port
        )
    except mariadb.Error as e:
        print(f"Error: {e}")
        return

    # create a cursor object using cursor() method
    cursor = conn.cursor()

    try:
        cursor.execute("SET FOREIGN_KEY_CHECKS=0;")
        cursor.execute(f"TRUNCATE TABLE {database_name}.classes;")
        cursor.execute(f"TRUNCATE TABLE {database_name}.voxels;")
        cursor.execute(f"TRUNCATE TABLE {database_name}.objects;")
        cursor.execute("SET FOREIGN_KEY_CHECKS=1;")
    except mariadb.Error as e:
        print(f"Error: {e}")
        conn.close()
        return
    
    print(database_name, "tables correctly truncated!")
    conn.close()

if __name__ == "__main__":
    main()