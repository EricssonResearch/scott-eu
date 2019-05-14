using System;
using System.Collections.Generic;
using System.Drawing;
using System.Text.RegularExpressions;
using System.Windows.Forms;

namespace PDDLPlanning
{
    public partial class Form1 : Form
    {
        private List<Predicate> previousPreds;
        private Action _selectedAction;
        private State _selectedState;
        private List<State> _states;
        private string _path_d = "";
        private string _path_p = "";
        private StateManager sm;
        public Form1()
        {
            InitializeComponent();
            // initPDDL();
            // InitializeComponent();
            // _states = sm.GetStates();
            // _selectedState = _states[0];
            // button3.Enabled = false;
            // previousPreds = new List<Predicate>();
        }

        // updating action and predicate lists
        private void UpdateForm()
        {
            _selectedState = sm.GetCurrentState();
            _states = sm.GetStates();
            _selectedAction = null;
            listView1.Items.Clear();
            string s;
            int counter = 0;
            foreach (Action a in sm.GetActions())
            {
                s = counter++ + ": " + a.Name + " ";
                a.actualParameters.ForEach(str => s += str + " ");
                listView1.Items.Add(s);
            }
            listView2.Clear();
            foreach (Predicate p in _selectedState.StateInfo)
            {
                if (!previousPreds.Contains(p))
                {
                    listView2.Items.Insert(0, p.Name + ": " + string.Join(" ", p.args)).ForeColor = Color.Red;
                }
                else
                    listView2.Items.Add(p.Name + ": " + string.Join(" ", p.args)).ForeColor = Color.Black;

            }
            foreach (Predicate p in previousPreds)
            {
                if (_selectedState.StateInfo.Contains(p))
                    continue;
                listView2.Items.Insert(0, p.Name + ": " + string.Join(" ", p.args)).ForeColor = Color.Gray;
                listView2.Items[0].Font = new Font(listView2.Font, FontStyle.Strikeout);
            }
            previousPreds = _selectedState.StateInfo;

        }

        // refreshing the state tree when an action is chosen (going to the new state)
        private void button3_Click(object sender, EventArgs e)
        {
            sm.PerformAction(_selectedAction);
            UpdateForm();
            
           //listView2.Items.Add(_selectedState.StateID.ToString());
            button2.Enabled = true;
            button3.Enabled = false;
            var text = updateStateTree(_selectedState);
            var pic = Graphviz.RenderImage(text, "jpg");
            pictureBox1.Image = pic;
        }

        // refreshing the state tree when undo button is clicked (going back to the previous state)
        private void button2_Click(object sender, EventArgs e)
        {
            sm.stepBack();
            UpdateForm();
            var text = updateStateTree(_selectedState);
            var pic = Graphviz.RenderImage(text, "jpg");
            pictureBox1.Image = pic;
        }

        // refreshing the state tree when an action is selected (coloring arrow coresponding to the selected action in blue)
        private void listView1_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (listView1.SelectedItems.Count == 0) return;

            _selectedAction = sm.GetActions()[listView1.SelectedIndices[0]];
            button3.Enabled = true;
            var text = updateStateTree(_selectedState);
            var pic = Graphviz.RenderImage(text, "jpg");
            pictureBox1.Image = pic;
        }

        // text box for domain file path
        private void textBox1_TextChanged(object sender, EventArgs e)
        {
            _path_d = textBox1.Text;
            button1.ForeColor = DefaultForeColor;
            button1.BackColor = DefaultBackColor;
            button1.Text = "Import";
        }

        // text box for problem file path
        private void textBox2_TextChanged(object sender, EventArgs e)
        {
            _path_p = textBox2.Text;
            button1.ForeColor = DefaultForeColor;
            button1.BackColor = DefaultBackColor;
            button1.Text = "Import";
        }

        // importing the files, checking if the right files are imported and creating a state tree
        private void button1_Click(object sender, EventArgs e)
        {
            if (_path_d.Trim(' ') == "" || _path_p.Trim(' ') == "")
            {
                button1.ForeColor = Color.White;
                button1.BackColor = Color.DarkRed;
                button1.Text = "No paths";
                return;
            }

            string input_d;
            string input_p;
            try
            {
                input_d = System.IO.File.ReadAllText(_path_d);
                input_p = System.IO.File.ReadAllText(_path_p);
            }
            catch (Exception)
            {
                button1.ForeColor = Color.White;
                button1.BackColor = Color.DarkRed;
                button1.Text = "Wrong paths";
                return;
            }

            try
            {
                readInput(input_d, input_p);
            }
            catch (Exception)
            {
                button1.ForeColor = Color.White;
                button1.BackColor = Color.DarkRed;
                button1.Text = "Wrong files";
                return;
            }

            _states = sm.GetStates();
            _selectedState = _states[0];
            button3.Enabled = false;
            previousPreds = new List<Predicate>();
            UpdateForm();
            var graphVizString = "digraph g{\n     node [margin=0.1 fontsize=12 width=0.2 shape=circle style=filled];" +
                                 "\n      A [color=red lp=\"2,1222!?\"];\n    A->B->C;\n    B->D;\n     D [color=blue]}";

            // Bitmap bm = new Bitmap(Graphviz.RenderImage(graphVizString, "jpg"));
            var text = updateStateTree(_selectedState);
            var pic = Graphviz.RenderImage(text, "jpg");
            pictureBox1.Image = pic;
            
            button1.Text = "Reset";
        }

        private State PreformAction(State state, Action action)
        {
            return _states[(_states.IndexOf(state) + 1) % _states.Count];
        }

        //hard coded domain and problem, no longer used.
        private void initPDDL()
        {

            //ProcessStartInfo startInfo = new ProcessStartInfo();
            //startInfo.CreateNoWindow = true;
            //startInfo.RedirectStandardOutput = true;
            //startInfo.UseShellExecute = false;
            //startInfo.FileName = "parser.exe";
            //startInfo.Arguments = "test-domain.pddl";
            //var proc = Process.Start(startInfo);
            //List<string> output = new List<string>();
            //while (!proc.StandardOutput.EndOfStream)
            //{
            //    output.Add(proc.StandardOutput.ReadLine());
            //}
            /*List<Predicate> preds = new List<Predicate>();
            preds.Add(new Predicate("room", 1));
            preds.Add(new Predicate("ball", 1));
            preds.Add(new Predicate("gripper", 1));
            preds.Add(new Predicate("at-robby", 1));
            preds.Add(new Predicate("at", 2));
            preds.Add(new Predicate("free", 1));
            preds.Add(new Predicate("carry", 2));
            //Create actions
            List<Action> acts = new List<Action>();
            acts.Add(new Action("move", 2));
            acts.Add(new Action("pick", 3));
            acts.Add(new Action("drop", 3));
            //preconditions
            //move ?from ?to
            acts[0].ActionParameters.Add("from");
            acts[0].ActionParameters.Add("to");
            acts[0].AddPrecondition(new Predicate("room", 1, "from"));
            acts[0].AddPrecondition(new Predicate("room", 1, "to"));
            acts[0].AddPrecondition(new Predicate("at-robby", 1, "from"));
            //pick ?obj ?room ?gripper
            acts[1].ActionParameters.Add("obj");
            acts[1].ActionParameters.Add("room");
            acts[1].ActionParameters.Add("gripper");
            acts[1].AddPrecondition(new Predicate("ball", 1, "obj"));
            acts[1].AddPrecondition(new Predicate("room", 1, "room"));
            acts[1].AddPrecondition(new Predicate("gripper", 1, "gripper"));
            acts[1].AddPrecondition(new Predicate("at", 2, "obj", "room"));
            acts[1].AddPrecondition(new Predicate("at-robby", 1, "room"));
            acts[1].AddPrecondition(new Predicate("free", 1, "gripper"));
            //drop ?obj ?room ?gripper
            acts[2].ActionParameters.Add("obj");
            acts[2].ActionParameters.Add("room");
            acts[2].ActionParameters.Add("gripper");
            acts[2].AddPrecondition(new Predicate("ball", 1, "obj"));
            acts[2].AddPrecondition(new Predicate("room", 1, "room"));
            acts[2].AddPrecondition(new Predicate("gripper", 1, "gripper"));
            acts[2].AddPrecondition(new Predicate("carry", 2, "obj", "gripper"));
            acts[2].AddPrecondition(new Predicate("at-robby", 1, "room"));

            //Create effects for the actions, first negative 
            //move
            acts[0].negativeEffects.Add(new Predicate("at-robby", 1, "from"));
            //pick
            acts[1].negativeEffects.Add(new Predicate("at", 2, "obj", "room"));
            acts[1].negativeEffects.Add(new Predicate("free", 1, "gripper"));
            //drop
            acts[2].negativeEffects.Add(new Predicate("carry", 2, "obj", "gripper"));
            //then positive
            //move
            acts[0].positiveEffects.Add(new Predicate("at-robby", 1, "to"));
            //pick
            acts[1].positiveEffects.Add(new Predicate("carry", 2, "obj", "gripper"));
            //drop
            acts[2].positiveEffects.Add(new Predicate("at", 2, "obj", "room"));
            acts[2].positiveEffects.Add(new Predicate("free", 1, "gripper"));

            //create the domain and problem files
            Domain d = new Domain(acts, preds);
            Problem p = new Problem(d);
            //adds objects to the problem file
            p.AddObj("rooma");
            p.AddObj("roomb");
            p.AddObj("b");
            p.AddObj("g");
            //Create the initial state
            List<string> objList = new List<string>();
            ////Initialize problem-class with p.AddInitState();
            p.AddInitState(new Predicate("room", 1, "rooma"));
            p.AddInitState(new Predicate("room", 1, "roomb"));
            p.AddInitState(new Predicate("ball", 1, "b"));
            p.AddInitState(new Predicate("gripper", 1, "g"));
            p.AddInitState(new Predicate("at", 2, "b", "rooma"));
            p.AddInitState(new Predicate("at-robby", 1, "rooma"));
            p.AddInitState(new Predicate("free", 1, "g"));

            sm = new StateManager(p);*/
        }

        
        private void pictureBox1_Click(object sender, EventArgs e)
        {

        }

        private string updateStateTree(State state)
        {
            var text = "digraph g{\n";
            const string indent = "  ";
            text += indent + "graph [splines=ortho];\n"+ indent + "node [margin=0.05 fontsize=8 width=0.01 shape=circle style=filled];\n" + indent + sm.GenerateGraphString(sm.rootState,_selectedAction) + indent + state.StateID + " [color=red];\n";

            text += "}";
            return text;
        }

        private int removeWhite(string input)
        {
            var whitespace = new Regex("^(\\r\\n| |\\r|\\n|\\t)");
            var lenght = 0;
            while (whitespace.IsMatch(input))
            {
                var value = whitespace.Match(input).Value;
                input = input.Substring(value.Length);
                lenght += value.Length;
            }

            return lenght;
        }


        // reads the .pddl file ignoring the define domain lines, and creates new predicates and actions from there.
        // uses regexes to match the names.
        private void readInput(string domain, string problem)
        {
            var name_Regex = new Regex("^[a-zA-Z]+[-a-zA-Z0-9]*");
            var name_Regex_question = new Regex("^\\?[a-zA-Z]+[-a-zA-Z0-9]*");
            var right_par = new Regex("^\\)");
            List<Predicate> preds = new List<Predicate>();
            List<Action> acts = new List<Action>();

            domain = domain.Substring(removeWhite(domain));
            var domain_start = new Regex("^\\(define *\\( *domain *[a-zA-Z]+[-a-zA-Z0-9]* *\\)").Match(domain).Value;
            if (domain_start == "")
            {
                throw new Exception();
            }

            domain = domain.Substring(domain_start.Length);
            domain = domain.Substring(removeWhite(domain));
            domain = domain.Substring(1);
            domain = domain.TrimEnd(' ', '\n', '\r', '\t');
            domain = domain.Remove(domain.Length - 1);
            while (domain.Length > 0)
            {
                domain = domain.Substring(removeWhite(domain));

                if (domain.StartsWith(":predicates"))
                {
                    domain = domain.Substring(":predicates".Length);
                    domain = domain.Substring(removeWhite(domain));
                    while (!right_par.IsMatch(domain))
                    {
                        domain = domain.Substring(1);
                        var name = name_Regex.Match(domain).Value;
                        domain = domain.Substring(name.Length);
                        domain = domain.Substring(removeWhite(domain));
                        var count = 0;
                        while (name_Regex_question.IsMatch(domain))
                        {
                            count++;
                            domain = domain.Substring(name_Regex_question.Match(domain).Value.Length);
                            domain = domain.Substring(removeWhite(domain));
                        }
                        domain = domain.Substring(1);
                        preds.Add(new Predicate(name, count));
                        domain = domain.Substring(removeWhite(domain));
                    }
                    domain = domain.Substring(1);
                    domain = domain.Substring(removeWhite(domain));
                    domain = domain.Length > 0 ? domain.Substring(1) : domain;
                }
                else if (domain.StartsWith(":action"))
                {
                    domain = domain.Substring(":action".Length);
                    domain = domain.Substring(removeWhite(domain));
                    var name = name_Regex.Match(domain).Value;
                    domain = domain.Substring(name.Length);
                    domain = domain.Substring(removeWhite(domain));
                    var action = new Action(name, 0);
                    if (domain.StartsWith(":parameters"))
                    {
                        domain = domain.Substring(":parameters".Length);
                        domain = domain.Substring(removeWhite(domain));
                        domain = domain.Substring(1);
                        domain = domain.Substring(removeWhite(domain));
                        var count = 0;

                        while (!domain.StartsWith(")"))
                        {
                            var par = name_Regex_question.Match(domain).Value;
                            domain = domain.Substring(par.Length);
                            action.ActionParameters.Add(par.TrimStart('?', ' ', '\n', '\r', '\t'));
                            count++;
                            domain = domain.Substring(removeWhite(domain));
                        }
                        domain = domain.Substring(1);
                        domain = domain.Substring(removeWhite(domain));
                        action.nrOfParams = count;
                    }

                    if (domain.StartsWith(":precondition"))
                    {
                        domain = domain.Substring(":precondition".Length);
                        domain = domain.Substring(removeWhite(domain));
                        domain = domain.Substring(1);
                        domain = domain.Substring(removeWhite(domain));
                        domain = domain.Substring(3);
                        domain = domain.Substring(removeWhite(domain));
                        while (!right_par.IsMatch(domain))
                        {
                            domain = domain.Substring(1);
                            var prec_name = name_Regex.Match(domain).Value;
                            
                            domain = domain.Substring(prec_name.Length);
                            domain = domain.Substring(removeWhite(domain));
                            var count = 0;
                            var names = new List<string>();
                            while (name_Regex_question.IsMatch(domain))
                            {
                                count++;
                                var n = name_Regex_question.Match(domain).Value;
                                domain = domain.Substring(n.Length);
                                names.Add(n.TrimStart('?', ' ', '\n', '\r', '\t'));
                                domain = domain.Substring(removeWhite(domain));
                            }
                            domain = domain.Substring(1);
                            domain = domain.Substring(removeWhite(domain));
                            action.AddPrecondition(new Predicate(prec_name, count, names));
                        }
                        domain = domain.Substring(1);
                        domain = domain.Substring(removeWhite(domain));
                    }

                    if (domain.StartsWith(":effect"))
                    {
                        domain = domain.Substring(":effect".Length);
                        domain = domain.Substring(removeWhite(domain));
                        domain = domain.Substring(1);
                        domain = domain.Substring(removeWhite(domain));
                        domain = domain.Substring(3);
                        domain = domain.Substring(removeWhite(domain));
                        while (!right_par.IsMatch(domain))
                        {
                            domain = domain.Substring(1);
                            domain = domain.Substring(removeWhite(domain));
                            var prec_name = name_Regex.Match(domain).Value;

                            domain = domain.Substring(prec_name.Length);
                            domain = domain.Substring(removeWhite(domain));
                            if (prec_name == "not")
                            {
                                while (!right_par.IsMatch(domain))
                                {
                                    domain = domain.Substring(1);
                                    domain = domain.Substring(removeWhite(domain));
                                    prec_name = name_Regex.Match(domain).Value;
                                    domain = domain.Substring(prec_name.Length);
                                    domain = domain.Substring(removeWhite(domain));

                                    var count_not = 0;
                                    var names_not = new List<string>();
                                    while (name_Regex_question.IsMatch(domain))
                                    {
                                        count_not++;
                                        var n = name_Regex_question.Match(domain).Value;
                                        domain = domain.Substring(n.Length);
                                        names_not.Add(n.TrimStart('?', ' ', '\n', '\r', '\t'));
                                        domain = domain.Substring(removeWhite(domain));
                                    }
                                    action.negativeEffects.Add(new Predicate(prec_name, count_not, names_not));
                                    domain = domain.Substring(1);
                                }
                                domain = domain.Substring(removeWhite(domain));
                                domain = domain.Substring(1);
                                domain = domain.Substring(removeWhite(domain));
                                continue;
                            }

                            var count = 0;
                            var names = new List<string>();
                            while (name_Regex_question.IsMatch(domain))
                            {
                                count++;
                                var n = name_Regex_question.Match(domain).Value;
                                domain = domain.Substring(n.Length);
                                names.Add(n.TrimStart('?', ' ', '\n', '\r', '\t'));
                                domain = domain.Substring(1);
                                domain = domain.Substring(removeWhite(domain));
                            }
                            domain = domain.Substring(removeWhite(domain));
                            action.positiveEffects.Add(new Predicate(prec_name, count, names));
                        }
                        domain = domain.Substring(1);
                        domain = domain.Substring(removeWhite(domain));
                        domain = domain.Substring(1);
                        domain = domain.Substring(removeWhite(domain));
                        domain = domain.Length > 0 ? domain.Substring(1) : domain;
                    }
                    acts.Add(action);
                }

                else
                {
                    throw new Exception();
                }

            }
            var d = new Domain(acts, preds);
            var p = new Problem(d);

            problem = problem.Substring(removeWhite(problem));
            var problem_start =
                new Regex(
                        "^\\(define\\s*\\(\\s*problem\\s*[a-zA-Z]+[-a-zA-Z0-9]*\\s*\\)\\s*\\(\\s*:domain\\s*[a-zA-Z]+[-a-zA-Z0-9]*\\s*\\)")
                    .Match(problem).Value;
            if (problem_start == "")
            {
                throw new Exception();
            }
            problem = problem.Substring(problem_start.Length);
            problem = problem.Substring(removeWhite(problem));
            problem = problem.Substring(1);
            problem = problem.TrimEnd(' ', '\n', '\r', '\t');
            problem = problem.Remove(problem.Length - 1);

            while (problem.Length > 0)
            {
                if (problem.StartsWith(":objects"))
                {
                    problem = problem.Substring(":objects".Length);
                    problem = problem.Substring(removeWhite(problem));

                    while (!problem.StartsWith(")"))
                    {
                        var obj = name_Regex.Match(problem).Value;
                        p.AddObj(obj);
                        problem = problem.Substring(obj.Length);
                        problem = problem.Substring(removeWhite(problem));
                    }
                    problem = problem.Substring(1);
                    problem = problem.Substring(removeWhite(problem));
                    problem = problem.Length > 0 ? problem.Substring(1) : problem;
                }

                else if (problem.StartsWith(":init"))
                {
                    problem = problem.Substring(":init".Length);
                    problem = problem.Substring(removeWhite(problem));

                    while (!problem.StartsWith(")"))
                    {
                        problem = problem.Substring(1);
                        var pred_name = name_Regex.Match(problem).Value;

                        problem = problem.Substring(pred_name.Length);
                        problem = problem.Substring(removeWhite(problem));

                        var arg_list = new List<string>();
                        while (!problem.StartsWith(")"))
                        {
                            var pred_arg = name_Regex.Match(problem).Value;
                            arg_list.Add(pred_arg);
                            problem = problem.Substring(pred_arg.Length);
                            problem = problem.Substring(removeWhite(problem));
                        }
                        p.AddInitState(new Predicate(pred_name, arg_list.Count, arg_list));
                        problem = problem.Substring(1);
                        problem = problem.Substring(removeWhite(problem));
                    }
                    problem = problem.Substring(1);
                    problem = problem.Substring(removeWhite(problem));
                    problem = problem.Length > 0 ? problem.Substring(1) : problem;
                }

                else if (problem.StartsWith(":goal"))
                {
                    problem = problem.Substring(":goal".Length);
                    problem = problem.Substring(removeWhite(problem));

                    while (!problem.StartsWith(")"))
                    {
                        problem = problem.Substring(1);
                        var pred_name = name_Regex.Match(problem).Value;

                        problem = problem.Substring(pred_name.Length);
                        problem = problem.Substring(removeWhite(problem));

                        var arg_list = new List<string>();
                        while (!problem.StartsWith(")"))
                        {
                            var pred_arg = name_Regex.Match(problem).Value;
                            arg_list.Add(pred_arg);
                            problem = problem.Substring(pred_arg.Length);
                            problem = problem.Substring(removeWhite(problem));
                        }
                        p.AddGoalState(new Predicate(pred_name, arg_list.Count, arg_list));
                        problem = problem.Substring(1);
                        problem = problem.Substring(removeWhite(problem));
                    }
                    problem = problem.Substring(1);
                    problem = problem.Substring(removeWhite(problem));
                    problem = problem.Length > 0 ? problem.Substring(1) : problem;
                }

                else
                {
                    throw new Exception();
                }
            }

            sm = new StateManager(p);
        }

        private void label1_Click(object sender, EventArgs e)
        {

        }
        private Point startingPoint = Point.Empty;
        private Point movingPoint = Point.Empty;
        private bool panning = false;

        void pictureBox1_MouseDown(object sender, MouseEventArgs e)
        {
            panning = true;
            startingPoint = new Point(e.Location.X - movingPoint.X,
                                      e.Location.Y - movingPoint.Y);
        }

        void pictureBox1_MouseUp(object sender, MouseEventArgs e)
        {
            panning = false;
        }

        void pictureBox1_MouseMove(object sender, MouseEventArgs e)
        {
            if (panning)
            {
                movingPoint = new Point(e.Location.X - startingPoint.X,
                                        e.Location.Y - startingPoint.Y);
                pictureBox1.Invalidate();
            }
        }

        void pictureBox1_Paint(object sender, PaintEventArgs e)
        {
            e.Graphics.Clear(Color.White);
            if (pictureBox1.Image != null)
                e.Graphics.DrawImage(pictureBox1.Image, movingPoint);
        }

        private void listView2_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void button4_Click(object sender, EventArgs e)
        {
            openFileDialog1.ShowDialog();
            textBox1.Text = openFileDialog1.FileName;
        }
        private void button5_Click(object sender, EventArgs e)
        {
            openFileDialog1.ShowDialog();
            textBox2.Text = openFileDialog1.FileName;
        }

    }
}
