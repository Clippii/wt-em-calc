import pickle


def clone(value):


    return pickle.loads(pickle.dumps(value,protocol=pickle.HIGHEST_PROTOCOL))
