class Action(object):
    """Abstract class for actions.

    Attributes
    ----------
    confidence : float
        Estimated quality of an action by the policy that produced it.
    metadata : dict
        Optional dictionary of metadata about the action.
    """
    def __init__(self,
                 confidence=0.0,
                 metadata=None):
        self.confidence = confidence
        self.metadata = metadata
        if self.metadata is None:
            self.metadata = {}

class NoAction(Action):
    """ Proxy for taking no action when none can be found! """
    pass
