import { useState, useEffect } from "react";
import { CreateToDo } from "./paths/createToDo";
import { DeleteToDo } from "./paths/deleteToDo";
import { UpdateToDo } from "./paths/updateToDo";

interface ToDo {
    id: number;
    message: string;
    time_input: string;
    due_date: string;
}

export const ToDos = () => {
    const [TodoList, setTodoList] = useState<ToDo[]>(() => {
        const saved = localStorage.getItem("todos");
        return saved ? JSON.parse(saved) : [];
    });
    const [UserMessage, setUserMessage] = useState("")
    const [EditingID, setEditingID] = useState(0)
    const [UpdateMessage, setUpdateMessage] = useState("")
    const [CompletingIDs, setCompletingIDs] = useState<number[]>([]);

    useEffect(() => {
        localStorage.setItem("todos", JSON.stringify(TodoList));
    }, [TodoList]);

    const submission = () => {
        if (UserMessage === "") {
            return
        } else {
            CreateToDo(UserMessage, setTodoList);
            setUserMessage("");
        }
    }

    const handleKeyDown = (event: React.KeyboardEvent<HTMLInputElement>) => {
        if (event.key === 'Enter') {
            submission();
        }
      };

    const handleDelete = (id: number) => {
        setCompletingIDs(prev => [...prev, id]);
        setTimeout(() => {
            DeleteToDo(id, setTodoList);
            setCompletingIDs(prev => prev.filter(i => i !== id));
        }, 400); // match the CSS transition duration
    };

    return (
        <>
        <div id="title"> To-Do List </div>
        <hr id="horLine"></hr>
            <div id="inputArea">
                <input id="inputField" value={UserMessage} placeholder="What's next?" onChange={(event) => setUserMessage(event.target.value)} onKeyDown={handleKeyDown}/>
                <button id="inputButton" onClick={submission}>SUBMIT</button>
            </div>
        <div id="listArea">
        <div id="listField">
            {TodoList.map((todoObject: ToDo) => (
                <div key={todoObject.id}>
                    <input
                    type="checkbox"
                    className="checkBox"
                    onChange={(e) => { if (e.target.checked) handleDelete(todoObject.id); }}
                    />
                    <span className={`todoItem${CompletingIDs.includes(todoObject.id) ? " completed" : ""}`}>
                    {todoObject.message}
                    </span>
                    {UpdateToDo(EditingID, todoObject.id, UpdateMessage, setTodoList, setEditingID, setUpdateMessage)}
                    <br></br>
                </div>
            ))}
        </div>
        </div>
        </>
    );
}



// todo List Array at the top
// useEffect -> that stores the array when a change is made
// Create Func
// Update Func
// Delete/Done Func
// Display