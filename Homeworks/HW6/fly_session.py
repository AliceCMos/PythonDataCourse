import datajoint as dj


# fail-safe user name retrieval
username = dj.conn().conn_info['user']
schema = dj.schema('{}_hw6'.format(username))
schema.drop(True)
schema = dj.schema('{}_hw6'.format(username))


# Table definitions

@schema
class Fly(dj.Manual):
    definition = """
    # Experimental animals
    fly_id         : int                          # Unique animal ID
    ---
    dob=null       : date                         # date of birth
    sex="U"        : enum('M','F','U')      # sex
    """


@schema
class Session(dj.Manual):
    definition = """
    # Experiment session
    -> Fly
    session_date               : date                         # date
    ---
    experiment_setup           : int                          # experiment setup ID
    experimenter               : varchar(100)                 # experimenter name
    data_path=''               : varchar(255)                 #
    """
    

# Insert the following data into the table
    
fly_data = [
 {'dob': "2020-03-01", 'fly_id': 0, 'sex': 'M'},
 {'dob': "2019-11-19", 'fly_id': 1, 'sex': 'M'},
 {'dob': "2019-11-20", 'fly_id': 2, 'sex': 'U'},
 {'dob': "2019-12-25", 'fly_id': 5, 'sex': 'F'},
 {'dob': "2020-01-01", 'fly_id': 10, 'sex': 'F'},
 {'dob': "2020-01-03", 'fly_id': 11, 'sex': 'F'},
 {'dob': "2020-05-12", 'fly_id': 100, 'sex': 'F'}
]

session_data = [
 {'experiment_setup': 0,
  'experimenter': 'gucky92',
  'fly_id': 0,
  'session_date': "2020-05-15",
  'data_path': 'imaging_data'
 },
 {'experiment_setup': 0,
  'experimenter': 'gucky92',
  'fly_id': 0,
  'session_date': "2020-05-19",
  'data_path': 'imaging_data'
 },
 {'experiment_setup': 1,
  'experimenter': 'jko14',
  'fly_id': 5,
  'session_date': "2020-01-05",
  'data_path': 'imaging_data'
 },
 {'experiment_setup': 100,
  'experimenter': 'jportes',
  'fly_id': 100,
  'session_date': "2020-05-25",
  'data_path': 'imaging_data'
 }
]

Fly.insert(fly_data, skip_duplicates=True)
Session.insert(session_data, skip_duplicates=True)