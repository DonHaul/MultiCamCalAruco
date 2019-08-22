


def extractKeyFromDictList(dictt,keyy):
    '''
    turns a specific key of a dictionary list into a list itself

    Args:
        dictt: dict to extract the key from
        keyy: key to turn into list
    Returns:
        [List]: list of the key
    '''
    return [ item[keyy] for item in dictt ]


def replicateThingInList(thing,N):
    '''
    makes copys of an element in the list

    Args:
        thing: thing to replicate
        N: name of replicas to make
    '''
    l =[]

    for i in range(N):
        l.append(thing)

    return l



def is_empty(any_structure):
    '''
    checks if something is empty or not
    if list is empty, or dict has no keys or something is None
    '''
    if any_structure:
        #print('Structure is not empty.')
        return False
    else:
        #print('Structure is empty.')
        return True


