import { Dispatch, SetStateAction } from "react"

interface ToDo {
    id: number;
    message: string;
    time_input: string;
    due_date: string;
}

export const UpdateToDo = (editingID: number, currentID: number, UpdateMessage: string, setTodoList: Dispatch<SetStateAction<ToDo[]>>, setEditingID: Dispatch<SetStateAction<number>>, setUpdateMessage: Dispatch<SetStateAction<string>>) => {

    if (editingID === currentID) {
        const submissionUpdate = () => {
            if (UpdateMessage === "") {
                return
            } else {
                setTodoList((prev: ToDo[]) => prev.map((todo) => todo.id === editingID ? { ...todo, message: UpdateMessage } : todo));
                setUpdateMessage("")
                setEditingID(0)
            }
        }

        const handleKeyDownUpdate = (event: React.KeyboardEvent<HTMLInputElement>) => {
            if (event.key === 'Enter') {
                submissionUpdate();
            }
          };

        return (
            <div>
                <input id="updateField" value={UpdateMessage} placeholder="Edit?" onChange={(event) => setUpdateMessage(event.target.value)} onKeyDown={handleKeyDownUpdate} />
                <button id="updateButton" onClick={submissionUpdate}>CHANGE</button>
            </div>
        );
    } else {
        return <button className="editButton" onClick={() => setEditingID(currentID)}>EDIT</button>
    }
}