



class DNA_Sequence(object):

    '''
        Stores an individuals gait parameters
             [BASE_STATE_X_DELTA, BASE_STATE_Y_DELTA, BASE_STATE_Z_DELTA, STEP_Z_MAX_HIEGHT, TORSO_SHIFT_DELTA, TORSO_LEFT_SHIFT, STEP_TIME, TORSO_SHIFT_TIME   ]
    '''

    def __init__(self, sequence):
        self.sequence = sequence

