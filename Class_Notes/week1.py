def Cel2Fahr(degC):
    degF = (degC * 9/5) + 32
    return(degF)

def passwordEval(username,password,login):
    """takes username and password as inputs, evaluates the pair and
    tells you if it is a correct combination from the login dict."""

    username = username.lower()
    password = password.lower()

    if username in login.keys():
        if login[username] == password:
            print("Username and password accepted!")
        else:
            print("Username/password not found!")

    elif username not in login.keys():
        print("Username/password not found!")
