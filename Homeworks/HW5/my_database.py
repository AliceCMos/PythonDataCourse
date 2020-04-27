import datajoint as dj

schema = dj.schema('homework2')


@schema
class FlySubject(dj.Manual):
    definition = """
    # some fly table
    subject_id : int  # id for fly subject
    ---
    age : float # age of fly in days
    sex = 'U': enum('F', 'M', 'U')  # sex of fly
    comments = null : varchar(4000)
    """
    
    
@schema
class Stimulus(dj.Manual):
    definition = """
    # stimulus table
    stimulus_name : varchar(31) # short name for stimulus
    ---
    stimulus_type : enum('full-field', 'grating', 'movie')
    duration : float # in seconds
    """
    
    
@schema
class RecordingSession(dj.Manual):
    definition = """
    # record them sessions
    -> FlySubject
    recording_id : int 
    ---
    -> Stimulus
    experimenter : varchar(127)
    recording_quality : enum('good', 'bad', 'ugly')
    comments = null : varchar(4000)
    """