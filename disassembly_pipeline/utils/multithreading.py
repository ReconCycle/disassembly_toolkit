from threading import Thread

def do_concurrently(ls, wait_until_task_completion = False):
    """To this function, you pass a list. Each element IN the list must be (function name, function_args). Then these functions will be run
    in separate threads, at once (concurrently). 
    WARNING/BUG - the args must not be named.
    
    Args:
        ls: list of [[function, function_args]]
            A list of grouped [function, function_args], for each a thread will be made.
        wait_until_task_completion: bool
            If true, wait for all threads to complete, with (thr.join())
    Returns:
        0
    Example:
    >>> do_concurrently([[panda_1.JMove, [(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5), 1.5]]] wait_until_task_completion = False)
    """
    assert type(ls) == list
    
    thrs = []
    for el in ls:
        thr = Thread(target=el[0], kwargs=el[1])
        thrs.append(thr)
    for thr in thrs:
        thr.start()
        
    if wait_until_task_completion:
        for thr in thrs:
            thr.join()

    return 0
