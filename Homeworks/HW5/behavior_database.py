# this code is to go into a python file

import datajoint as dj

schema = dj.schema('behavior_experiment')

@schema
class Mouse(dj.Manual):
    definition = """
    # a table of all mice used for behavior
    subject_id: int # unique ID for this mouse
    ---
    dob: date
    sex = 'U': enum('F','M', 'U') # sex of the mouse
    strain = 'B6': enum('IT-Cre', 'PT-Cre', 'B6')
    """

@schema
class Task(dj.Manual):
    definition = """
    # task type
    task_name: varchar(31)
    ---
    target_size=null: int # size of target in mm
    target_position=null: enum('left','right')
    """

@schema
class Setup(dj.Manual):
    definition = """
    # setup for training
    setup_id: int
    ---
    setup_type: enum('TeenScience', 'MaxonEpos')
    setup_location: enum('BehaviorRoom', '2PhotonRoom')
    """

@schema
class BehaviorSession(dj.Manual):
    definition = """
    # behavior session
    -> Mouse
    session_id: int
    ---
    -> Task
    -> Setup
    date = CURRENT_TIMESTAMP: timestamp
    experimenter: varchar(127)
    study_name: varchar(127)
    max_rewards: int
    rewards_achieved: int
    comments = null: varchar(2000)
    """
