import datajoint as dj
import pandas as pd
import os


schema = dj.schema('lecture6_part2')

# auto-initialize whole schema
schema.drop(True)
schema = dj.schema('lecture6_part2')
    
    
@schema
class RecordingSession(dj.Manual):
    definition = """
    # record them sessions
    recording_id : int 
    ---
    filename : varchar(127)
    experimenter : varchar(127)
    recording_quality : enum('good', 'bad', 'ugly')
    comments = null : varchar(4000)
    """


@schema
class ImportedRecording(dj.Imported):
    definition = """
    -> RecordingSession
    ---
    data : blob@localstore
    """
    
    def make(self, key):
        print(key)
        filename = (RecordingSession & key).fetch1()['filename']
        data = pd.read_csv(os.path.join('data', filename+'.csv'))
        
        # key is a dictionary
        key.update({'data':data})
        
        self.insert1(key)
    

@schema
class AnalyzedRecording(dj.Computed):
    definition = """
    -> ImportedRecording
    
    ---
    mean : longblob
    """
    
    def make(self, key):
        print(key)
        # calculate mean for data in entry fetched
        mean = (ImportedRecording & key).fetch1()['data'].mean()
        
        key.update({'mean': mean})
        
        self.insert1(key)