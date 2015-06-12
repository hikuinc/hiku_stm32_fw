Dim StdIn, StdOut
Set StdIn = WScript.StdIn
Set StdOut = WScript.StdOut
StdOut.Write "const stm32_fw = """
If WScript.Arguments.Count = 0 Then
Do While Not StdIn.AtEndOfStream
StdOut.Write "\x"
StdOut.Write Right("0"+Hex(Asc(StdIn.Read(1))),2)
Loop
End If
StdOut.Write """;"
