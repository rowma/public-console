import React, { useEffect } from 'react';
import './App.css';

import { createMuiTheme, makeStyles, ThemeProvider, Theme, createStyles } from '@material-ui/core/styles';
import { green } from '@material-ui/core/colors';

import { VariableSizeList } from 'react-window';

import AppBar from '@material-ui/core/AppBar';
import Container from '@material-ui/core/Container';
import Toolbar from '@material-ui/core/Toolbar';
import Typography from '@material-ui/core/Typography';
import Paper from '@material-ui/core/Paper';
import Grid from '@material-ui/core/Grid';
import Box from '@material-ui/core/Box';
import Link from '@material-ui/core/Link';
import TextField from '@material-ui/core/TextField';
import InputLabel from '@material-ui/core/InputLabel';
import MenuItem from '@material-ui/core/MenuItem';
import Select from '@material-ui/core/Select';

import Radio from '@material-ui/core/Radio';
import RadioGroup from '@material-ui/core/RadioGroup';
import FormControlLabel from '@material-ui/core/FormControlLabel';
import FormControl from '@material-ui/core/FormControl';
import Button from '@material-ui/core/Button';
import CircularProgress from '@material-ui/core/CircularProgress';

import Table from '@material-ui/core/Table';
import TableBody from '@material-ui/core/TableBody';
import TableCell from '@material-ui/core/TableCell';
import TableContainer from '@material-ui/core/TableContainer';
import TableRow from '@material-ui/core/TableRow';

import AceEditor from "react-ace";
import "ace-builds/src-noconflict/mode-python";
import "ace-builds/src-noconflict/theme-github";

// @ts-ignore
import Rowma from 'rowma_js';

const theme = createMuiTheme({
  palette: {
    primary: {
      main: '#ffffff',
    },
    secondary: {
      main: '#38B48B',
    },
    contrastThreshold: 3,
    tonalOffset: 0.2,
  },
  typography: {
    button: {
      textTransform: 'none'
    }
  }
});

const useStyles = makeStyles((theme: Theme) => (
  createStyles({
    root: {
      flexGrow: 1,
    },
    menuButton: {
      marginRight: theme.spacing(2),
    },
    title: {
      flexGrow: 1,
    },
    paper: {
      padding: theme.spacing(2),
      textAlign: 'center',
      color: theme.palette.text.secondary,
      background: '#fdfdfd'
    },
    radioButtons: {
      maxHeight: 300,
      minHeight: 300,
      maxWidth: '100%',
      textAlign: 'center',
      overflow: 'auto'
    },
    info: {
      padding: theme.spacing(2),
      textAlign: 'center',
      color: theme.palette.text.secondary,
      background: '#f6f6f6'
    },
    header: {
      color: theme.palette.text.primary,
      background: '#fcfcfc'
    },
    footer: {
      textAlign: 'left',
    },
    footerLink: {
      color: '#38B48B',
      marginRight: '1rem',
    },
    radioGroup: {
      textAlign: 'left',
    },
    textField: {
      width: '60%',
    },
    buttonProgress: {
      color: green[500],
      position: 'absolute',
      top: '50%',
      left: '50%',
      marginTop: -12,
      marginLeft: -12,
    },
    formControl: {
      marginBottom: theme.spacing(3),
      margin: theme.spacing(1),
      minWidth: 120,
    },
  })
));

const sleep = (ms: number) => {
  return new Promise(resolve => setTimeout(resolve, ms));
}

interface NetworkInformationInterface {
  name: string;
  type: string;
  location: string;
  owner: string;
  version: string;
  url: string;
}

const emptyNetworkInformation: NetworkInformationInterface = {
  name: '',
  type: '',
  location: '',
  owner: '',
  version: '',
  url: ''
}

const App: React.FC = () => {
  const [rowmaUrl, setRowmaUrl] = React.useState<string>("https://rowma.moriokalab.com");
  const [rowma, setRowma] = React.useState<any>(null);
  const [robotUuids, setRobotUuids] = React.useState<Array<string> | undefined>(undefined);
  const [selectedRobot, setSelectedRobot] = React.useState<any | null>(null);
  const [rosrunCommands, setRosrunCommands] = React.useState<Array<string>>([]);
  const [selectedRosrunCommand, setSelectedRosrunCommand] = React.useState<string>('');
  const [roslaunchCommands, setRoslaunchCommands] = React.useState<Array<string>>([]);
  const [rosnodes, setRosnodes] = React.useState<Array<string>>([]);
  const [rostopics, setRostopics] = React.useState<Array<string>>([]);
  const [topicMsg, setTopicMsg] = React.useState<string>('');
  const [selectedRoslaunchCommand, setSelectedRoslaunchCommand] = React.useState<string>('');
  const [selectedRosnode, setSelectedRosnode] = React.useState<string>('');
  const [selectedRostopic, setSelectedRostopic] = React.useState<string>('');
  const [selectedRostopicForPublish, setSelectedRostopicForPublish] = React.useState<string>('');
  const [selectedRostopicForUnsubscribe, setSelectedRostopicForUnsubscribe] = React.useState<string>('');
  const [selectedR2rRostopic, setSelectedR2rRostopic] = React.useState<string>('');
  const [selectedDestinationRobot, setSelectedDestinationRobot] = React.useState<string>('');
  const [robot, setRobot] = React.useState<any>({});
  const [submitUrlButtonLoading, setSubmitUrlButtonLoading] = React.useState<boolean>(false);
  const [connectButtonLoading, setConnectButtonLoading] = React.useState<boolean>(false);
  const [rosrunButtonLoading, setRosrunButtonLoading] = React.useState<boolean>(false);
  const [roslaunchButtonLoading, setRoslaunchButtonLoading] = React.useState<boolean>(false);
  const [rosnodeButtonLoading, setRosnodeButtonLoading] = React.useState<boolean>(false);
  const [rostopicButtonLoading, setRostopicButtonLoading] = React.useState<boolean>(false);
  const [rostopicForPublishButtonLoading, setRostopicForPublishButtonLoading] = React.useState<boolean>(false);
  const [rostopicForUnsubscribeButtonLoading, setRostopicForUnsubscribeButtonLoading] = React.useState<boolean>(false);
  const [addScriptButtonLoading, setAddScriptButtonLoading] = React.useState<boolean>(false);

  const [networkInformation, setNetworkInformation] = React.useState<any>(emptyNetworkInformation);
  const [items, setItems] = React.useState<Array<string>>([]);
  const [_roslaunchLog, setRoslaunchLog] = React.useState<Array<string>>([]);
  const [roslaunchLogs, setRoslaunchLogs] = React.useState<Array<string>>([]);
  const [rosrunLogs, setRosrunLogs] = React.useState<Array<string>>([]);
  const [scriptName, setScriptName] = React.useState<string>("");
  const [script, setScript] = React.useState<string>("");

  const classes = useStyles();

  const handleUrlFieldChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setRowmaUrl((event.target as HTMLInputElement).value);
  }

  const handleConnectNetworkClick = async () => {
    setSubmitUrlButtonLoading(true);
    const _rowma = new Rowma({ baseURL: rowmaUrl });
    setRowma(_rowma);

    const networkInfo = await _rowma.getNetworkInformation()
    setNetworkInformation({ url: rowmaUrl, ...networkInfo.data })

    const connList = await _rowma.currentConnectionList()
    setRobotUuids(connList.data.map((robot: any) => robot.uuid));

    setRobot({})
    setRosrunCommands([]);
    setRoslaunchCommands([]);
    setRostopics([]);

    setSubmitUrlButtonLoading(false);
  }

  const handleRobotChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setSelectedRobot((event.target as HTMLInputElement).value);
  };

  const handleTopicArrival = (event: any) => {
    setItems(items => [...items, JSON.stringify(event.msg)])
  }

  const handleRoslaunchLog = (event: any) => {
    console.log(event)
    setRoslaunchLogs(logs => [...logs, JSON.stringify(event)])
    // setRoslaunchLog(items => [...items, JSON.stringify(event.log)])
  }

  const handleRosrunLog = (event: any) => {
    console.log(event)
    setRosrunLogs(logs => [...logs, JSON.stringify(event)])
    // setRosrunLog(items => [...items, JSON.stringify(event.log)])
  }

  const handleConnectClicked = () => {
    setConnectButtonLoading(true);
    rowma.connect().catch((e: any) => {
      console.error(e)
    })
    rowma.setRobotUuid(selectedRobot);

    rowma.getRobotStatus(selectedRobot).then((res: any) => {
      setRobot(res.data)
      setRosnodes(res.data.rosnodes)
      setRosrunCommands(res.data.rosrunCommands);
      setRoslaunchCommands(res.data.launchCommands);
      setRostopics(res.data.rostopics);
      setConnectButtonLoading(false);
    })
  }

  const handleRosrunCommandChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setSelectedRosrunCommand((event.target as HTMLInputElement).value);
  };

  const handleRoslaunchCommandChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setSelectedRoslaunchCommand((event.target as HTMLInputElement).value);
  };

  const handleRosrunButtonClick = async () => {
    setRosrunButtonLoading(true);
    setRosnodeButtonLoading(true);
    const rosrunArgs = '';
    await rowma.runRosrun(selectedRobot, selectedRosrunCommand, rosrunArgs);
    setRosrunButtonLoading(false);
    await sleep(2500);
    const _robot = await rowma.getRobotStatus("", selectedRobot)
    setRosnodes(_robot.data.rosnodes)
    setRosnodeButtonLoading(false);
  }

  const handleRoslaunchButtonClick = async () => {
    setRoslaunchButtonLoading(true);
    setRosnodeButtonLoading(true);
    const result = await rowma.runLaunch(selectedRobot, selectedRoslaunchCommand)
    setRoslaunchButtonLoading(false);
    rowma.socket.on('roslaunch_log', handleRoslaunchLog)
    await sleep(2500);
    const _robot = await rowma.getRobotStatus("", selectedRobot)
    setRosnodes(_robot.data.rosnodes)
    setRosnodeButtonLoading(false);
  }

  const handleRosnodeChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setSelectedRosnode((event.target as HTMLInputElement).value);
  }

  const handleRosnodeButtonClick = async () => {
    setRosnodeButtonLoading(true);
    rowma.socket.on('rosrun_log', handleRosrunLog)
    const result = await rowma.killNodes(selectedRobot, [selectedRosnode]);
    if (result.status === 'success') {
      const index = rosnodes.indexOf(selectedRosnode)
      rosnodes.splice(index, 1);
    }
    setRosnodeButtonLoading(false);
  }

  const handleSubscribeButtonClick = async () => {
    setRostopicButtonLoading(true);
    await rowma.subscribe(selectedRostopic, handleTopicArrival)
    await rowma.setTopicRoute(selectedRobot, 'application', rowma.uuid, selectedRostopic);
    setRostopicButtonLoading(false);
  }

  const handleRostopicChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setSelectedRostopic((event.target as HTMLInputElement).value);
  }

  const Row = ({ index, style }: any) => (
    <div style={style}>
      <span className="p-4">{items[index]}</span>
    </div>
  );

  const RoslaunchLogRow = ({ index, style }: any) => (
    <div style={style}>
      <span className="p-4 text-gray-200">{JSON.parse(roslaunchLogs[index]).log}</span>
    </div>
  );

  const RosrunLogRow = ({ index, style }: any) => (
    <div style={style}>
      <span className="p-4 text-gray-200">{JSON.parse(rosrunLogs[index]).log}</span>
    </div>
  );

  const getItemSize = (index: number) => {
    return items[index] && items[index].length > 60 ? 70 : 30
  }

  const getRoslaunchLogItemSize = (index: number) => {
    return roslaunchLogs[index] && roslaunchLogs[index].length > 60 ? 50 : 20
  }

  const getRosrunLogItemSize = (index: number) => {
    return rosrunLogs[index] && rosrunLogs[index].length > 60 ? 50 : 20
  }

  const ListComponent = () => (
    <VariableSizeList
      height={300}
      width={'95%'}
      itemCount={items.length}
      itemSize={getItemSize}
      className="border text-left mt-4 mx-4"
      initialScrollOffset={items.length * 70}
    >
      {Row}
    </VariableSizeList>
  );

  const RoslaunchLogListComponent = () => (
    <VariableSizeList
      height={300}
      width={'95%'}
      itemCount={roslaunchLogs.length}
      itemSize={getRoslaunchLogItemSize}
      className="border text-left my-4 mx-4 bg-gray-700"
      initialScrollOffset={roslaunchLogs.length * 70}
    >
      {RoslaunchLogRow}
    </VariableSizeList>
  );

  const RosrunLogListComponent = () => (
    <VariableSizeList
      height={300}
      width={'95%'}
      itemCount={rosrunLogs.length}
      itemSize={getRosrunLogItemSize}
      className="border text-left my-4 mx-4 bg-gray-700"
      initialScrollOffset={rosrunLogs.length * 70}
    >
      {RosrunLogRow}
    </VariableSizeList>
  );

  const handleUnsubscribeButtonClick = () => {
    setRostopicForUnsubscribeButtonLoading(true);
    rowma.unsubscribeTopic(selectedRobot, selectedRostopicForUnsubscribe)
    setRostopicForUnsubscribeButtonLoading(false);
  }

  const handlePublishButtonClick = async () => {
    setRostopicForPublishButtonLoading(true);
    await rowma.publish(selectedRobot, selectedRostopicForPublish, JSON.parse(topicMsg))
    setRostopicForPublishButtonLoading(false);
  }

  const handleTopicMsgChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setTopicMsg((event.target as HTMLInputElement).value);
  }

  const handlePublishRostopicChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setSelectedRostopicForPublish((event.target as HTMLInputElement).value);
  }

  const handleRostopicUnsubscribeChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setSelectedRostopicForUnsubscribe((event.target as HTMLInputElement).value)
  }

  const handleTopicSelectboxChange = (event: React.ChangeEvent<{ value: unknown }>) => {
    setSelectedR2rRostopic(event.target.value as string);
  };

  const handleDestinationSelectboxChange = (event: React.ChangeEvent<{ value: unknown }>) => {
    setSelectedDestinationRobot(event.target.value as string);
  };

  const handleSubscribeR2rButtonClick = () => {
    rowma.setTopicRoute(selectedRobot, 'robot', selectedDestinationRobot, selectedR2rRostopic)
  }

  const handleScriptNameFieldChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setScriptName((event.target as HTMLInputElement).value);
  }

  const handleEditorChange = (newValue: string) => {
    setScript(newValue)
  }

  const handleUploadScriptButtonClick = async () => {
    setAddScriptButtonLoading(true)
    await rowma.addScript(selectedRobot, scriptName, script)
    setAddScriptButtonLoading(false)
  }

  return (
    <div className={`${classes.root} App`}>
      <ThemeProvider theme={theme}>
        <AppBar position="static" className={classes.header}>
          <Toolbar>
            <Container className="text-left">
              <img src="/public-console/logo.png" width="28" height="28" className="inline mx-4" />
              <Typography variant="h5" className="inline align-middle">Rowma Network Console</Typography>
            </Container>
          </Toolbar>
        </AppBar>
        <Container>
          <Grid container spacing={3} className="py-8">
            <Grid item xs={12} sm={12} md={12}>
              <Paper className={classes.paper}>
                <div className="flex items-center justify-center">
                  <TextField color="secondary" margin="dense" label="Network URL" variant="outlined" className={classes.textField} onChange={handleUrlFieldChange} value={rowmaUrl} />
                  <div className="relative mx-4">
                    <Button
                      variant="contained"
                      color="primary"
                      disabled={submitUrlButtonLoading}
                      onClick={handleConnectNetworkClick}
                    >
                      Connect
                    </Button>
                    {submitUrlButtonLoading && <CircularProgress size={24} className={classes.buttonProgress} />}
                  </div>
                </div>
              </Paper>
            </Grid>

            <Grid item xs={12} sm={12} md={4}>
              <Paper className={classes.paper}>
                <div>
                  <FormControl component="fieldset" className={classes.radioButtons}>
                    <div className="my-4">
                      <Typography variant='h5'>Select Your Robot{"'"}s UUID</Typography>
                    </div>
                    {(!robotUuids || (robotUuids && robotUuids.length === 0)) &&
                      <p>Robot not found...</p>
                    }
                    <RadioGroup aria-label="robots" name="robots" value={selectedRobot} onChange={handleRobotChange} className={classes.radioGroup}>
                    {robotUuids && robotUuids.map(uuid => {
                      return (
                        <FormControlLabel value={uuid} control={<Radio />} label={uuid} />
                      )
                    })}
                    </RadioGroup>
                  </FormControl>
                </div>
                <div className="relative">
                  <Button
                    variant="contained"
                    color="primary"
                    disabled={connectButtonLoading || !selectedRobot}
                    onClick={handleConnectClicked}
                  >
                    Connect
                  </Button>
                  {connectButtonLoading && <CircularProgress size={24} className={classes.buttonProgress} />}
                </div>
              </Paper>
            </Grid>

            <Grid item xs={12} sm={12} md={8}>
              <Paper className={`${classes.paper} h-full`}>
                <div className="mx-4 mt-4">
                  <iframe width={560} height={315} src="https://www.youtube.com/embed/7Rl0_7da47k" data-frameborder="0" data-allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" data-allowfullscreen></iframe>
                </div>
              </Paper>
            </Grid>

            <Grid item xs={12} sm={12} md={4}>
              <Paper className={classes.paper}>
                <div>
                  <FormControl component="fieldset" className={classes.radioButtons}>
                    <div className="my-4">
                      <Typography variant='h5'>Select a rosrun command</Typography>
                    </div>
                    <RadioGroup aria-label="rosrun" name="rosrun" value={selectedRosrunCommand} onChange={handleRosrunCommandChange} className={classes.radioGroup}>
                    {rosrunCommands && rosrunCommands.map(command => {
                      return (
                        <FormControlLabel value={command} control={<Radio />} label={command} />
                      )
                    })}
                    </RadioGroup>
                  </FormControl>
                </div>
                <div className="relative">
                  <Button
                    variant="contained"
                    color="primary"
                    disabled={rosrunButtonLoading || selectedRosrunCommand === ''}
                    onClick={handleRosrunButtonClick}
                  >
                    Execute
                  </Button>
                  {rosrunButtonLoading && <CircularProgress size={24} className={classes.buttonProgress} />}
                </div>
              </Paper>
            </Grid>

            <Grid item xs={12} sm={12} md={8}>
              <Paper className={classes.paper}>
                <RosrunLogListComponent />
              </Paper>
            </Grid>

            <Grid item xs={12} sm={12} md={4}>
              <Paper className={classes.paper}>
                <div>
                  <FormControl component="fieldset" className={classes.radioButtons}>
                    <div className="my-4">
                      <Typography variant='h5'>Select a roslaunch command</Typography>
                    </div>
                    <RadioGroup aria-label="roslaunch" name="roslaunch" value={selectedRoslaunchCommand} onChange={handleRoslaunchCommandChange} className={classes.radioGroup}>
                    {roslaunchCommands && roslaunchCommands.map(command => {
                      return (
                        <FormControlLabel value={command} control={<Radio />} label={command} />
                      )
                    })}
                    </RadioGroup>
                  </FormControl>
                </div>
                <div className="relative">
                  <Button
                    variant="contained"
                    color="primary"
                    disabled={roslaunchButtonLoading || selectedRoslaunchCommand === ''}
                    onClick={handleRoslaunchButtonClick}
                  >
                    Execute
                  </Button>
                  {roslaunchButtonLoading && <CircularProgress size={24} className={classes.buttonProgress} />}
                </div>
              </Paper>
            </Grid>

            <Grid item xs={12} sm={12} md={8}>
              <Paper className={classes.paper}>
                <RoslaunchLogListComponent />
              </Paper>
            </Grid>

            <Grid item xs={12} sm={12} md={12}>
              <Paper className={classes.paper}>
                <div>
                  <Typography variant='h5'>Add Script</Typography>
                  <div className="my-4"><span>Caution: There is a possibility of arbitrary code execution when you use script addition feature, therefore, you should watch out testing this feature. Add <b>ENABLE_SCRIPT_DOWNLOAD=True</b> to rosrun rowma_ros rowma command if you use this feature.</span></div>
                </div>

                <div className="flex items-center justify-center">
                  <TextField color="secondary" margin="dense" label="Script Name*" variant="outlined" className={classes.textField} onChange={handleScriptNameFieldChange} value={scriptName} />
                </div>

                <AceEditor
                  mode="python"
                  theme="github"
                  value={script}
                  onChange={handleEditorChange}
                  name="public-console-editor"
                  width="100%"
                  fontSize={18}
                  highlightActiveLine={false}
                  className="bg-gray-700 text-gray-200"
                  editorProps={{ $blockScrolling: true }}
                />

                <div className="relative mt-4">
                  <Button
                    variant="contained"
                    color="primary"
                    disabled={roslaunchButtonLoading || script === '' || scriptName === ''}
                    onClick={handleUploadScriptButtonClick}
                  >
                    Upload
                  </Button>
                  {addScriptButtonLoading && <CircularProgress size={24} className={classes.buttonProgress} />}
                </div>
              </Paper>
            </Grid>

            <Grid item xs={12} sm={12} md={4}>
              <Paper className={classes.paper}>
                <div>
                  <FormControl component="fieldset" className={classes.radioButtons}>
                    <div className="my-4">
                      <Typography variant='h5'>Running ROS nodes</Typography>
                    </div>
                    <RadioGroup aria-label="rosnodes" name="rosnodes" value={selectedRosnode} onChange={handleRosnodeChange} className={classes.radioGroup}>
                    {rosnodes && rosnodes.map((node: any) => {
                      return (
                        <FormControlLabel value={node} control={<Radio />} label={node} />
                      )
                    })}
                    </RadioGroup>
                  </FormControl>
                </div>
                <div className="relative">
                  <Button
                    variant="contained"
                    color="primary"
                    disabled={rosnodeButtonLoading || selectedRosnode === ''}
                    onClick={handleRosnodeButtonClick}
                  >
                    Kill
                  </Button>
                  {rosnodeButtonLoading && <CircularProgress size={24} className={classes.buttonProgress} />}
                </div>
              </Paper>
            </Grid>

            <Grid item xs={12} sm={12} md={8}>
              <Paper className={classes.paper}>
                <div className="flex">
                  <div className="w-1/3">
                    <FormControl component="fieldset" className={classes.radioButtons}>
                      <div className="my-4">
                        <Typography variant='h5'>Subscribe rostopic</Typography>
                      </div>
                      <RadioGroup aria-label="rostopics" name="rostopics" value={selectedRostopic} onChange={handleRostopicChange} className={classes.radioGroup}>
                        {rostopics && rostopics.map((topic: any) => {
                          return (
                            <FormControlLabel value={topic} control={<Radio />} label={topic} />
                          )
                        })}
                      </RadioGroup>
                    </FormControl>
                    <div className="relative">
                      <Button
                        variant="contained"
                        color="primary"
                        disabled={rostopicButtonLoading || selectedRostopic === ''}
                        onClick={handleSubscribeButtonClick}
                      >
                        Subscribe
                      </Button>
                    </div>
                  </div>
                  <div className="w-2/3">
                    <ListComponent />
                  </div>
                </div>
              </Paper>
            </Grid>

            <Grid item xs={12} sm={12} md={4}>
              <Paper className={classes.paper}>
                <div>
                  <FormControl component="fieldset" className={classes.radioButtons}>
                    <div className="my-4">
                      <Typography variant='h5'>Unsubscribe rostopic</Typography>
                    </div>
                    <RadioGroup aria-label="rostopics" name="rostopics" value={selectedRostopicForUnsubscribe} onChange={handleRostopicUnsubscribeChange} className={classes.radioGroup}>
                    {rostopics && rostopics.map((topic: any) => {
                      return (
                        <FormControlLabel value={topic} control={<Radio />} label={topic} />
                      )
                    })}
                    </RadioGroup>
                  </FormControl>
                </div>
                <div className="relative">
                  <Button
                    variant="contained"
                    color="primary"
                    disabled={rostopicForUnsubscribeButtonLoading || selectedRostopicForUnsubscribe === ''}
                    onClick={handleUnsubscribeButtonClick}
                  >
                    Unsubscribe
                  </Button>
                </div>
              </Paper>
            </Grid>

            <Grid item xs={12} sm={12} md={8}>
              <Paper className={classes.paper}>
                <div className="flex">
                  <div className="w-1/3">
                    <FormControl component="fieldset" className={classes.radioButtons}>
                      <div className="my-4">
                        <Typography variant='h5'>Publish rostopic</Typography>
                      </div>
                      <RadioGroup aria-label="rostopics" name="rostopics" value={selectedRostopicForPublish} onChange={handlePublishRostopicChange} className={classes.radioGroup}>
                        {rostopics && rostopics.map((topic: any) => {
                          return (
                            <FormControlLabel value={topic} control={<Radio />} label={topic} />
                          )
                        })}
                      </RadioGroup>
                    </FormControl>
                    <div className="relative">
                      <Button
                        variant="contained"
                        color="primary"
                        disabled={rostopicForPublishButtonLoading || selectedRostopicForPublish === ''}
                        onClick={handlePublishButtonClick}
                      >
                        Publish
                      </Button>
                    </div>
                  </div>
                  <div className="w-2/3">
                    <TextField
                      className="border"
                      label="Write your topic"
                      multiline
                      rows="15"
                      placeholder='{"data": "This is a sample for std_msgs/String"}'
                      color="secondary"
                      fullWidth
                      variant="outlined"
                      onChange={handleTopicMsgChange}
                      value={topicMsg}
                    />
                  </div>
                </div>
              </Paper>
            </Grid>
            <Grid item xs={12}>
              <Paper className={classes.paper}>
                <div className="flex content-center items-center justify-center">
                  <span>Send</span>
                  <FormControl className={classes.formControl}>
                    <InputLabel>Topic</InputLabel>
                    <Select
                      labelId="demo-simple-select-label"
                      value={selectedR2rRostopic}
                      onChange={handleTopicSelectboxChange}
                    >
                      {rostopics && rostopics.map((topic: any) => (
                        <MenuItem value={topic}>{topic}</MenuItem>
                      ))}
                    </Select>
                  </FormControl>
                  <span> from {selectedRobot} to </span>
                  <FormControl className={classes.formControl}>
                    <InputLabel>Destination</InputLabel>
                    <Select
                      labelId="demo-simple-select-label"
                      value={selectedDestinationRobot}
                      onChange={handleDestinationSelectboxChange}
                    >
                      {robotUuids && robotUuids.map((uuid: any) => (
                        <MenuItem value={uuid}>{uuid}</MenuItem>
                      ))}
                    </Select>
                  </FormControl>
                  <div className="relative">
                    <Button
                      variant="contained"
                      color="primary"
                      disabled={selectedR2rRostopic === ''}
                      onClick={handleSubscribeR2rButtonClick}
                    >
                      Subscribe
                    </Button>
                  </div>
                </div>
              </Paper>
            </Grid>

            <Grid item xs={12}>
              <Paper className={classes.info}>
                <div className="my-4">
                  <Typography variant="h6">Network Information</Typography>
                </div>
                <Grid
                  container
                  direction="row"
                  justify="center"
                  alignItems="center"
                >
                  <Grid item xs={12} sm={12} md={6}>
                    <TableContainer className="pb-4">
                      <Table aria-label="simple table">
                        <TableBody>
                          <TableRow>
                            <TableCell scope="row">Network Name</TableCell>
                            <TableCell align="right">{networkInformation.name}</TableCell>
                          </TableRow>
                          <TableRow>
                            <TableCell scope="row">Network Type</TableCell>
                            <TableCell align="right">{networkInformation.type}</TableCell>
                          </TableRow>
                          <TableRow>
                            <TableCell scope="row">Network URL</TableCell>
                            <TableCell align="right">{networkInformation.url}</TableCell>
                          </TableRow>
                          <TableRow>
                            <TableCell scope="row">Network Location</TableCell>
                            <TableCell align="right">{networkInformation.location}</TableCell>
                          </TableRow>
                          <TableRow>
                            <TableCell scope="row">Network Owner</TableCell>
                            <TableCell align="right">{networkInformation.owner}</TableCell>
                          </TableRow>
                          <TableRow>
                            <TableCell scope="row">Network Version</TableCell>
                            <TableCell align="right">{networkInformation.version}</TableCell>
                          </TableRow>

                        </TableBody>
                      </Table>
                    </TableContainer>
                  </Grid>
                </Grid>
              </Paper>
            </Grid>

            <Grid item xs={12}>
              <Box className={classes.footer} fontSize={16}>
                <Link className={classes.footerLink} href="">How to Use This Page</Link>
                <Link className={classes.footerLink} href="https://rowma.github.io/documentation/en/getting-started">Documentation</Link>
                <Link className={classes.footerLink} href="https://github.com/rowma/rowma">GitHub</Link>
              </Box>
            </Grid>

          </Grid>
        </Container>
      </ThemeProvider>
    </div>
  );
}

export default App
