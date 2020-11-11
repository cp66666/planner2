### cmdrmonitor 
----
##### Brief
A monitor for publisheres and subscribers in unique participant.
##### 

##### Usage
You can build it locally or copy the built cmdrmonitor exe file to your local bin. 
Then you can do as follows:
```bash
$ cmdrmonitor #This will list all the pubs and subs in default domain 100.
$ cmdrmonitor 102 #This will list all the pubs and subs in domain 102.
$ cmdrmonitor -h #This will shouw usage help.
```
##### Issues
Sometimes it has a latency when a subscriber or publisehr leaves the participant. However, new join action will be discovered at once. This will be fixed in a later time.